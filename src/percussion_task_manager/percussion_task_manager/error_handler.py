"""
error_handler.py
----------------
Centralised error handling for the percussion task manager.

The task manager's state machine routes every failure through TaskState.ERROR
with a human-readable string in `self._error_message`. This module parses that
string, classifies it into an `ErrorCategory`, and dispatches a `RecoveryAction`
(retry capture, reconnect, move home, safety-stop, …).

Each recovery primitive is a small, independent method on `ErrorHandler` so new
categories or actions can be added without touching the dispatcher.

Recovery budget
---------------
Each category has its own retry counter, indexed by `ErrorCategory`. Once the
counter exceeds `MAX_RETRIES[category]`, the action escalates to SAFETY_STOP.
Counters are reset by `reset_retries()` (called when the state machine returns
to IDLE).

Adding a new error category
---------------------------
1. Add a value to `ErrorCategory`.
2. Add a `(keywords, category)` row to `_CLASSIFICATION_TABLE` (first match wins).
3. Add a row to `CATEGORY_TO_ACTION` and optionally to `MAX_RETRIES`.
4. If you need a brand-new recovery action, add a value to `RecoveryAction`
   and an `elif` branch in `_dispatch()`.
"""

from __future__ import annotations

from enum import Enum
from typing import Dict, Optional, Tuple

from percussion_interfaces.srv import Reconnect


# ---------------------------------------------------------------------------
# Categories & actions
# ---------------------------------------------------------------------------

class ErrorCategory(str, Enum):
    MARKER_NOT_FOUND  = 'MARKER_NOT_FOUND'
    CAMERA_FAULT      = 'CAMERA_FAULT'
    ROBOT_CONNECTION  = 'ROBOT_CONNECTION'
    MOTION_TIMEOUT    = 'MOTION_TIMEOUT'
    MOTION_FAILURE    = 'MOTION_FAILURE'
    INDUCTIVE_FAULT   = 'INDUCTIVE_FAULT'
    ARDUINO_FAULT     = 'ARDUINO_FAULT'
    CONFIG_FAULT      = 'CONFIG_FAULT'
    UNKNOWN           = 'UNKNOWN'


class RecoveryAction(str, Enum):
    RETRY_CAPTURE        = 'RETRY_CAPTURE'
    RECONNECT_AND_RETRY  = 'RECONNECT_AND_RETRY'
    MOVE_HOME_AND_RETRY  = 'MOVE_HOME_AND_RETRY'
    SAFETY_STOP          = 'SAFETY_STOP'
    RETURN_TO_IDLE       = 'RETURN_TO_IDLE'
    NONE                 = 'NONE'


# ---------------------------------------------------------------------------
# Classification + policy tables
# ---------------------------------------------------------------------------

# Ordered classification table — first matching row wins.
# Each row is a tuple of (required_keywords, category). All keywords in the
# tuple must appear (case-insensitive) in the error message for the row to
# match. More specific rows should come first.
_CLASSIFICATION_TABLE: Tuple[Tuple[Tuple[str, ...], ErrorCategory], ...] = (
    # ---- Specific: motion timeouts before generic motion failures ----
    (('CONTACT', 'TIMEOUT'),       ErrorCategory.MOTION_TIMEOUT),
    (('FORCE',   'TIMEOUT'),       ErrorCategory.MOTION_TIMEOUT),
    (('MOTION',  'TIMEOUT'),       ErrorCategory.MOTION_TIMEOUT),
    (('TIMEOUT',),                 ErrorCategory.MOTION_TIMEOUT),

    # ---- Markers / vision ----
    (('NO MARKERS',),              ErrorCategory.MARKER_NOT_FOUND),
    (('MARKER', 'NOT'),            ErrorCategory.MARKER_NOT_FOUND),
    (('VISION',),                  ErrorCategory.MARKER_NOT_FOUND),

    # ---- Camera ----
    (('REALSENSE',),               ErrorCategory.CAMERA_FAULT),
    (('CAMERA',),                  ErrorCategory.CAMERA_FAULT),
    (('CAPTURE',),                 ErrorCategory.CAMERA_FAULT),

    # ---- Robot / motion server connectivity ----
    (('RTDE',),                    ErrorCategory.ROBOT_CONNECTION),
    (('ROBOT', 'CONNECT'),         ErrorCategory.ROBOT_CONNECTION),
    (('MOTION', 'SERVER'),         ErrorCategory.ROBOT_CONNECTION),
    (('MOTION', 'NOT ACCEPTED'),   ErrorCategory.ROBOT_CONNECTION),

    # ---- Inductive sensors ----
    (('INDUCTIVE',),               ErrorCategory.INDUCTIVE_FAULT),
    (('IND_VALUES',),              ErrorCategory.INDUCTIVE_FAULT),

    # ---- Arduino / serial ----
    (('ARDUINO',),                 ErrorCategory.ARDUINO_FAULT),
    (('HAMMER',),                  ErrorCategory.ARDUINO_FAULT),
    (('SERIAL',),                  ErrorCategory.ARDUINO_FAULT),

    # ---- Generic motion (catch-all) ----
    (('MOTION',),                  ErrorCategory.MOTION_FAILURE),

    # ---- Configuration / mode ----
    (('MODE',),                    ErrorCategory.CONFIG_FAULT),
    (('CONFIG',),                  ErrorCategory.CONFIG_FAULT),
)


# First-line recovery action per category. Once the per-category retry budget
# is exhausted, the dispatcher escalates to SAFETY_STOP regardless.
CATEGORY_TO_ACTION: Dict[ErrorCategory, RecoveryAction] = {
    ErrorCategory.MARKER_NOT_FOUND:  RecoveryAction.MOVE_HOME_AND_RETRY,
    ErrorCategory.CAMERA_FAULT:      RecoveryAction.RECONNECT_AND_RETRY,
    ErrorCategory.ROBOT_CONNECTION:  RecoveryAction.RECONNECT_AND_RETRY,
    ErrorCategory.MOTION_TIMEOUT:    RecoveryAction.MOVE_HOME_AND_RETRY,
    ErrorCategory.MOTION_FAILURE:    RecoveryAction.SAFETY_STOP,
    ErrorCategory.INDUCTIVE_FAULT:   RecoveryAction.MOVE_HOME_AND_RETRY,
    ErrorCategory.ARDUINO_FAULT:     RecoveryAction.RECONNECT_AND_RETRY,
    ErrorCategory.CONFIG_FAULT:      RecoveryAction.RETURN_TO_IDLE,
    ErrorCategory.UNKNOWN:           RecoveryAction.SAFETY_STOP,
}

# Max retries per category before escalating to SAFETY_STOP.
MAX_RETRIES: Dict[ErrorCategory, int] = {
    ErrorCategory.MARKER_NOT_FOUND:  2,
    ErrorCategory.CAMERA_FAULT:      2,
    ErrorCategory.ROBOT_CONNECTION:  2,
    ErrorCategory.MOTION_TIMEOUT:    2,
    ErrorCategory.MOTION_FAILURE:    0,
    ErrorCategory.INDUCTIVE_FAULT:   2,
    ErrorCategory.ARDUINO_FAULT:     2,
    ErrorCategory.CONFIG_FAULT:      0,
    ErrorCategory.UNKNOWN:           0,
}

# Seconds to wait before retrying a USB / serial peripheral.
_USB_RETRY_DELAY_SEC = 10.0


# ---------------------------------------------------------------------------
# Handler
# ---------------------------------------------------------------------------

class ErrorHandler:
    """
    Centralised error router. Holds a reference to the TaskManagerNode so each
    recovery primitive can drive the state machine, re-issue service calls, or
    queue new motion sequences via the existing node API.
    """

    def __init__(self, node) -> None:
        self._node = node
        self._retry_counts: Dict[ErrorCategory, int] = {}

        # Client for motion-node's RTDE reconnect service. Created lazily on
        # the parent node so it is destroyed with the node.
        self._reconnect_client = node.create_client(
            Reconnect, '/percussion/motion/reconnect'
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def handle_error(self, error_message: str,
                     context: Optional[dict] = None) -> RecoveryAction:
        """
        Classify `error_message`, pick a recovery action, dispatch it.

        Returns the action that was chosen (after retry-budget escalation).
        """
        category = self._classify(error_message)
        action   = self._choose_action(category)
        self._log(error_message, category, action, context)
        self._dispatch(action, category, context)
        return action

    def reset_retries(self) -> None:
        """Clear all per-category retry counters. Called when the state
        machine returns to IDLE so each new task starts with a fresh budget."""
        if self._retry_counts:
            self._node.get_logger().debug(
                f'Resetting error retry counters: {dict(self._retry_counts)}'
            )
        self._retry_counts.clear()

    # ------------------------------------------------------------------
    # Classification + policy
    # ------------------------------------------------------------------

    def _classify(self, error_message: str) -> ErrorCategory:
        msg = (error_message or '').upper()
        for keywords, category in _CLASSIFICATION_TABLE:
            if all(kw in msg for kw in keywords):
                return category
        return ErrorCategory.UNKNOWN

    def _choose_action(self, category: ErrorCategory) -> RecoveryAction:
        budget    = MAX_RETRIES.get(category, 0)
        attempts  = self._retry_counts.get(category, 0)
        if attempts >= budget:
            return RecoveryAction.SAFETY_STOP
        self._retry_counts[category] = attempts + 1
        return CATEGORY_TO_ACTION.get(category, RecoveryAction.SAFETY_STOP)

    def _log(self, message: str, category: ErrorCategory,
             action: RecoveryAction, context: Optional[dict]) -> None:
        attempts = self._retry_counts.get(category, 0)
        budget   = MAX_RETRIES.get(category, 0)
        ctx_str  = f' context={context}' if context else ''
        self._node.get_logger().error(
            f'[ErrorHandler] message={message!r} '
            f'category={category.value} action={action.value} '
            f'attempt={attempts}/{budget}{ctx_str}'
        )

    # ------------------------------------------------------------------
    # Dispatch
    # ------------------------------------------------------------------

    def _dispatch(self, action: RecoveryAction,
                  category: ErrorCategory,
                  context: Optional[dict]) -> None:
        if action is RecoveryAction.SAFETY_STOP:
            self._safety_stop()
        elif action is RecoveryAction.RETURN_TO_IDLE:
            self._return_to_idle()
        elif action is RecoveryAction.RETRY_CAPTURE:
            self._retry_capture()
        elif action is RecoveryAction.MOVE_HOME_AND_RETRY:
            self._move_to_home_and_retry()
        elif action is RecoveryAction.RECONNECT_AND_RETRY:
            self._attempt_reconnect(category, context)
        elif action is RecoveryAction.NONE:
            pass
        else:
            self._node.get_logger().error(
                f'[ErrorHandler] Unknown recovery action: {action}; '
                f'falling back to SAFETY_STOP.'
            )
            self._safety_stop()

    # ------------------------------------------------------------------
    # Recovery primitives
    # ------------------------------------------------------------------

    def _safety_stop(self) -> bool:
        """Halt the task: drop pending sequence, stay in ERROR for manual
        intervention. The motion node has already aborted by this point."""
        self._node.get_logger().error(
            '[ErrorHandler] SAFETY_STOP — pending sequence cleared. '
            'Manual intervention required; call /start_task to recover.'
        )
        self._node._sequence = []
        self._node._on_sequence_done = None
        self._node._pause_handler = None
        return True

    def _return_to_idle(self) -> bool:
        """Drop pending work and return the state machine to IDLE without
        moving the robot. Used for config-level faults where retrying is
        meaningless until the operator fixes the input."""
        from .task_manager_node import TaskState
        self._node.get_logger().warn(
            '[ErrorHandler] RETURN_TO_IDLE — clearing task and parking in IDLE.'
        )
        self._node._sequence = []
        self._node._on_sequence_done = None
        self._node._pause_handler = None
        self._node.publish_state(TaskState.IDLE)
        return True

    def _retry_capture(self) -> bool:
        """Re-issue the capture service from the current pose."""
        self._node.get_logger().warn('[ErrorHandler] RETRY_CAPTURE — re-running capture service.')
        try:
            self._node.run_capture_service()
            return True
        except Exception as exc:
            self._node.get_logger().error(f'[ErrorHandler] retry_capture failed: {exc}')
            return False

    def _move_to_home_and_retry(self) -> bool:
        """Run the active mode's return sequence, then re-home and re-capture.

        We use `build_return_sequence` instead of `build_home_sequence` because
        the arm may be mid-task; the return sequence is designed to retract
        safely from any in-task pose before joint-moving home.
        """
        from .task_manager_node import TaskState

        mode = self._node._mode
        if mode is None:
            self._node.get_logger().error(
                '[ErrorHandler] MOVE_HOME_AND_RETRY requested without an '
                'active mode; falling back to SAFETY_STOP.'
            )
            return self._safety_stop()

        self._node.get_logger().warn(
            f'[ErrorHandler] MOVE_HOME_AND_RETRY — running {mode.get("name")} '
            f'return sequence, then re-capturing.'
        )

        # After the return sequence completes, transition back to TASK_REQUESTED
        # so the normal state machine re-runs the home sequence + capture path.
        self._node._sequence       = mode['build_return_sequence']()
        self._node._pause_handler  = None
        self._node._on_sequence_done = (
            lambda: self._node.publish_state(TaskState.TASK_REQUESTED)
        )
        self._node._execute_next_step()
        return True

    def _attempt_reconnect(self, category: ErrorCategory,
                           context: Optional[dict]) -> bool:
        """
        Try to re-establish connectivity for `category`, then resume.

        - ROBOT_CONNECTION: call the motion node's `/reconnect` service
          (closes + re-opens RTDE), then re-publish the state that was in
          flight so the failed motion step re-issues.
        - CAMERA_FAULT:     wait 10 s, then re-trigger capture. The perception
          node opens a fresh RealSense pipeline per capture, so this is
          effectively a USB reconnect attempt.
        - ARDUINO_FAULT:    wait 10 s, then re-publish the state that was in
          flight (arduino_bridge opens the serial port per command).
        """
        if category is ErrorCategory.ROBOT_CONNECTION:
            return self._reconnect_rtde(context)
        if category is ErrorCategory.CAMERA_FAULT:
            return self._wait_then(_USB_RETRY_DELAY_SEC, self._retry_capture)
        if category is ErrorCategory.ARDUINO_FAULT:
            return self._wait_then(_USB_RETRY_DELAY_SEC,
                                   lambda: self._resume_after_reconnect(context))

        # Should not happen — RECONNECT_AND_RETRY was selected for an
        # unexpected category. Be safe.
        self._node.get_logger().warn(
            f'[ErrorHandler] No reconnect path for category {category.value}; '
            'falling back to SAFETY_STOP.'
        )
        return self._safety_stop()

    # ------------------------------------------------------------------
    # Reconnect helpers
    # ------------------------------------------------------------------

    def _reconnect_rtde(self, context: Optional[dict]) -> bool:
        """Call /percussion/motion/reconnect asynchronously; on success
        re-publish the state that was in flight so the state machine
        re-issues the failed motion step."""
        if not self._reconnect_client.service_is_ready():
            self._node.get_logger().warn(
                '[ErrorHandler] /percussion/motion/reconnect not ready; '
                'waiting briefly and retrying capture instead.'
            )
            return self._wait_then(_USB_RETRY_DELAY_SEC,
                                   lambda: self._resume_after_reconnect(context))

        self._node.get_logger().warn(
            '[ErrorHandler] Calling /percussion/motion/reconnect …'
        )
        future = self._reconnect_client.call_async(Reconnect.Request())

        def _on_response(fut) -> None:
            try:
                resp = fut.result()
            except Exception as exc:
                self._node.get_logger().error(
                    f'[ErrorHandler] reconnect service call raised: {exc}'
                )
                self._safety_stop()
                return
            if resp is None or not resp.success:
                msg = resp.message if resp else 'no response'
                self._node.get_logger().error(
                    f'[ErrorHandler] RTDE reconnect failed: {msg}'
                )
                self._safety_stop()
                return
            self._node.get_logger().info(
                f'[ErrorHandler] RTDE reconnect succeeded: {resp.message}'
            )
            self._resume_after_reconnect(context)

        future.add_done_callback(_on_response)
        return True

    def _resume_after_reconnect(self, context: Optional[dict]) -> bool:
        """Re-publish the pre-ERROR state so the state machine re-issues the
        failed step. Falls back to MOVE_HOME_AND_RETRY if the previous state
        is unknown."""
        from .task_manager_node import TaskState

        prev = (context or {}).get('previous_state')
        if prev is None:
            prev = getattr(self._node, '_previous_state', None)
        if not prev or prev == TaskState.ERROR.value:
            self._node.get_logger().warn(
                '[ErrorHandler] No previous state to resume from; '
                'falling back to MOVE_HOME_AND_RETRY.'
            )
            return self._move_to_home_and_retry()

        try:
            state = TaskState(prev)
        except ValueError:
            self._node.get_logger().warn(
                f'[ErrorHandler] Unrecognised previous state {prev!r}; '
                'falling back to MOVE_HOME_AND_RETRY.'
            )
            return self._move_to_home_and_retry()

        self._node.get_logger().info(
            f'[ErrorHandler] Resuming task by re-publishing state {state.value}.'
        )
        # Force a real state transition: the state-change callback dedupes on
        # _last_processed_state, and that value is currently 'ERROR'. Setting
        # it back to ERROR here means the resume publish actually fires the
        # state machine logic for `state`.
        self._node._last_processed_state = TaskState.ERROR.value
        self._node.publish_state(state)
        return True

    # ------------------------------------------------------------------
    # Misc utilities
    # ------------------------------------------------------------------

    def _wait_then(self, seconds: float, fn) -> bool:
        """Schedule `fn()` to be invoked once after `seconds`. Non-blocking;
        uses a rclpy one-shot timer."""
        self._node.get_logger().info(
            f'[ErrorHandler] Waiting {seconds:.0f}s before retry …'
        )
        timer_ref = {'handle': None}

        def _cb() -> None:
            handle = timer_ref['handle']
            if handle is not None:
                try:
                    handle.cancel()
                    self._node.destroy_timer(handle)
                except Exception:
                    pass
            try:
                fn()
            except Exception as exc:
                self._node.get_logger().error(
                    f'[ErrorHandler] delayed callback raised: {exc}'
                )

        timer_ref['handle'] = self._node.create_timer(seconds, _cb)
        return True
