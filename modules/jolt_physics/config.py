def can_build(env, platform):
    return not env["disable_physics_3d"]

def get_opts(platform):
    from SCons.Variables import BoolVariable

    return [
        BoolVariable("jolt_enable_asserts", "Enable assert debug output", False),
        BoolVariable("jolt_enable_asserts_mutex", "Enable assert mutex checks", True),
        BoolVariable("jolt_enable_trace", "Enable trace debug output. May be needed for some assert outputs.", False),
        BoolVariable("jolt_trace_vehicle", "Enable vehicle statistic tracing", False),
    ]


def configure(env):
    pass
