from prpy.exceptions import PrPyException


class AdaPyException(PrPyException):
    pass


def find_adapy_resource(relative_path, package='adapy'):
    from catkin.find_in_workspaces import find_in_workspaces

    paths = find_in_workspaces(project=package, search_dirs=['share'],
                               path=relative_path, first_match_only=True)

    if paths and len(paths) == 1:
        return paths[0]
    else:
        raise IOError('Loading AdaPy resource "{:s}" failed.'.format(
                      relative_path))
