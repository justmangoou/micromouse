"""Keep the unused navigation implementation out of the firmware build."""
Import("env")


def select_navigation(build_env, node):
    from pathlib import Path

    source = Path(node.srcnode().get_abspath())
    if source.parent != Path(env["PROJECT_DIR"]) / "lib" / "nav" / "src":
        return node
    excluded = ("geometry.cpp", "planner.cpp", "navigator.cpp") if env["PIOENV"] == "micromouse" else ("floodfill.cpp",)
    return None if source.name in excluded else node


env.AddBuildMiddleware(select_navigation)
