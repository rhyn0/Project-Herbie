import inspect

"""
Class made to help with usability of classes. Help Document for Herbie code is very out of date as of 6/1/2021
"""


def is_relevant(obj):
    """Filter for the inspector to filter out non user defined functions/classes"""
    if hasattr(obj, "__name__") and obj.__name__ == "type":
        return False
    if inspect.isfunction(obj) or inspect.isclass(obj) or inspect.ismethod(obj):
        return True


def print_docs(module):
    data = []
    for child in inspect.getmembers(module, is_relevant):

        doc = inspect.getdoc(child[1])
        if doc:
            data.append(str(child[0]) + "\n" + str(doc) + "\n\n")
            # print(child[0], doc, sep = '\n', end='\n\n')
    return data


if __name__ == "__main__":
    import commands
    import motor

    fn = "Manual.txt"
    with open(fn, "w") as fp:
        for pack in [commands, motor]:
            fp.writelines(print_docs(pack))

