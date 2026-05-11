from typing import List
from percussion_interfaces.msg import Pose6D


def _make_pose6d(x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0) -> Pose6D:
    p = Pose6D()
    p.x, p.y, p.z = x, y, z
    p.rx, p.ry, p.rz = rx, ry, rz
    return p


# ──────────────────────────────────────────────────────────────────────────
# FIXING MODE
# ──────────────────────────────────────────────────────────────────────────

def _fixing_home() -> List[dict]:
    """1-step home sequence for FIXING mode."""
    return [
        {
            'motion_type':  'JOINT_MOVE',
            'goal_Q':   [-2.2192, -1.841, 2.4652, -3.0588, -2.5319, -1.6911],
        },
    ]


def _fixing_sequence(marker_pose: List) -> List[dict]:
    """15-step sequence for FIXING mode."""
    return [
        {
            'motion_type':    'MOVE_TO_MARKER',
            'marker_pose':    marker_pose,
            'approach_offset': [0.05, 0.0, 0.0, 0.0, 0.0, 0.0],
        },
        {
            'motion_type':    'MOVE_TO_CONTACT', # Touch Marker facing
            'direction': [0.020, 0.0, 0.0, 0.0, 0.0, 0.0],
        },
        {
            'motion_type': 'PAUSE',
        },
        {
            'motion_type':    'RELATIVE_MOVE',
            'relative_pose': [0, 0.15, -0.05, 0, 0.0, 0.0],
        },
        {
            'motion_type':    'RELATIVE_MOVE',
            'relative_pose': [0.0, 0.0, 0.0, 0.0, -1.5701, 0.0],
        },
        {
            'motion_type':    'RELATIVE_MOVE',
            'relative_pose': [0.080, -0.08, 0, 0, 0.0, 0.0],
        },
        {
            'motion_type': 'MOVE_TO_CONTACT', # Touch ledger top down
            'direction': [0.0, 0.00707, -0.00707, 0.0, 0.0, 0.0],
        },
        {
            'motion_type':    'RELATIVE_MOVE',
            'relative_pose': [0, 0.08, 0, 0.0, 0.0, 0.0],
        },
        {
            'motion_type':    'RELATIVE_MOVE',
            'relative_pose': [0.0320, 0.0, 0.11, 0.0, 0.0, 0.0],
        },
        {
            'motion_type':    'MOVE_TO_CONTACT', # Touch bar facing
            'direction': [0.0, -0.00707, -0.00707, 0.0, 0.0, 0.0],
        },
        {
            'motion_type':    'RELATIVE_MOVE',
            'relative_pose': [0.0, 0.0, -0.06, -0.05, 0.0, 0.0],
        },
        {
            'motion_type':    'RELATIVE_MOVE',
            'relative_pose': [0.0, -0.05, 0.0, 0.0, 0.0, 0.0],
        },
        {
            'motion_type':    'RELATIVE_MOVE',
            'relative_pose': [0.0, 0.015, 0.045, 0.0, 0.0, 0.0],
        },
        {
            'motion_type':    'RELATIVE_MOVE',
            'relative_pose': [0.0, 0.010, 0.0, 0.0, 0.0, 0.0],
        },
        {
            'motion_type':    'RELATIVE_MOVE',
            'relative_pose': [0.0, -0.00, 0.015, 0.0, 0.0, 0.0],
        },
        {
            'motion_type': 'MOVE_TO_FORCE',
            'direction': [0.0, -0.05, 0.0, 0.0, 0.0, 0.0],
            'force_threshold': 15.0,
        },
        # {
            # 'motion_type':    'RELATIVE_MOVE',
            # 'relative_pose': [0.0, -0.007, 0.0, 0.0, 0.0, 0.0],
        # },
        
    ]


def _fixing_return() -> List[dict]:
    """3-step return sequence for FIXING mode."""
    return [
        {
            'motion_type':    'RELATIVE_MOVE',
            'relative_pose': [-0.10, 0.05, -0.10, 0.0, 0.0, 0.0],
        },
        #_fixing_home(),
        {
            'motion_type':  'JOINT_MOVE',
            'goal_Q':   [-2.2192, -1.841, 2.4652, -3.0588, -2.5319, -1.6911],
        },

    ]


# ──────────────────────────────────────────────────────────────────────────
# LOOSENING MODE
# ──────────────────────────────────────────────────────────────────────────

def _loosening_home() -> List[dict]:
    """2-step home sequence for LOOSENING mode."""
    return [
        #{
        #    'motion_type':    'RETURN_HOME',
        #    'home_pose':    [0.012859383601227098, 0.48737578483556077, 1.1524734298815074, -1.933597011391449, -1.1267506947795167, -1.9533873019226058],
        #},
        #{
        #    'motion_type': 'RELATIVE_MOVE',
        #    'relative_pose': [0.0, 0.0, -0.10, 3.10, 0.0, 0.0],
        #},
        {
            'motion_type':    'JOINT_MOVE',
            #'home_pose':    [0.1530, 0.4098, 0.5720, 2.4704, 0.7305, 1.4247],
            'goal_Q': [1.4552586078643799, -1.428090201025345, -2.368151903152466, -2.969842573205465, -3.7495289937794496e-05, -1.47551108995546514],
        },
        {
            'motion_type':     'JOINT_MOVE',
            'goal_Q': [1.4644263982772827, -0.7090473932078858, -1.802927017211914, -0.6575923723033448, -0.3903558889972132, 0.079482],
        }
    ]


def _loosening_sequence(marker_pose: Pose6D) -> List[dict]:
    """7-step sequence for LOOSENING mode."""
    return [
        {
            'motion_type':    'MOVE_TO_MARKER', # Move in front of marker
            'marker_pose':    marker_pose,
            'approach_offset': [0.05, 0.0, 0.0, 0.0, 1.57, -1.57],
            'invert_tcp':     True,
        },
        {
            'motion_type':    'MOVE_TO_CONTACT', # Touch marker forwards
            'direction': [0.020, 0.0, 0.0, 0.0, 0.0, 0.0],
        },
        {
            'motion_type':    'RELATIVE_MOVE', # Move back for clearance
            'relative_pose': [0.0, 0.15, -0.10, 0.0, 0.0, 0.0],
        },
        {
            'motion_type': 'RELATIVE_MOVE', # rotate 90° For correct position. 
            'relative_pose':[0.0, 0.0, 0.0, 0.0, 1.57, 0.0],
            #'Q_near': [0.7658, -2.4929, -1.0438, -4.9712, -1.0623, -0.6307],
            #'Q_near': [0.7746564745903015, -2.3317557773985804, -1.3908557891845703, -4.710584541360372, -1.1626928488360804, -0.6768596808062952],
        },
        {
            'motion_type':    'RELATIVE_MOVE', # Move in position for 2nd contact 
            'relative_pose': [-0.120, 0.0,  0.0, 0.0, 0.0, 0.0],
        },
        {
            'motion_type': 'MOVE_TO_CONTACT', # Touh bottom up
            'direction': [0.0, -0.020, 0.020, 0.0, 0.0, 0.0],
        },
        {
            'motion_type': 'RELATIVE_MOVE', # Create vertical space
            'relative_pose': [0.0, 0.120, 0.0, 0.0, 0.0, 0.0],
        },
        {
            'motion_type': 'RELATIVE_MOVE', # Move closer to pole
            'relative_pose': [-0.026, 0.0, 0.200, 0.0, 0.0, 0.0],
        },
        {
            'motion_type': 'MOVE_TO_CONTACT', # Touch pole facing
            'direction': [0.0, -0.00707, -0.00707, 0.0, 0.0, 0.0],
        },
        {
            'motion_type': 'RELATIVE_MOVE',
            'relative_pose': [0.0, -0.030, -0.0105, 0.0, 0.0, 0.0],
        },
        {
            'motion_type': 'MOVE_TO_FORCE',
            'direction': [0.0, -0.05, 0.0, 0.0, 0.0, 0.0],
            'force_threshold': 15.0,
        },
    ]


def _loosening_return() -> List[dict]:
    """3-step return sequence for LOOSENING mode."""
    return [
        {
            'motion_type':    'RELATIVE_MOVE',
            'relative_pose': [0.10, 0.05, -0.10, 0.0, 0.0, 0.0],
        },
        _loosening_home()[0],
    ]


# ──────────────────────────────────────────────────────────────────────────
# MODES REGISTRY
# ──────────────────────────────────────────────────────────────────────────

MODES = {
    'FIXING': {
        'build_home_sequence':   _fixing_home,
        'build_sequence':        _fixing_sequence,
        'build_return_sequence': _fixing_return,
        'arduino': {'msg_type': 'HAMMER_REQ', 'data': '8', 'msg_info': 'a'},
    },
    'LOOSENING': {
        'build_home_sequence':   _loosening_home,
        'build_sequence':        _loosening_sequence,
        'build_return_sequence': _loosening_return,
        'arduino': {'msg_type': 'HAMMER_REQ', 'data': '5', 'msg_info': 'a'},
    },
}
