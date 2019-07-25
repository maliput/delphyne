import py_trees.common
import py_trees.decorators

'''
The decorator can do something in the child with any object that
has a method called do_action with one parameter. This decorator
will feed the object with the decorated member variable.
It will always have the same status as the child and it will terminate
if the child status is different from RUNNING or SUCCESS.
'''
class AdditionalAction(py_trees.decorators.Decorator):

    def __init__(self, *, child, conditional_object,
                name=py_trees.common.Name.AUTO_GENERATED,
                ):
        super().__init__(child, name)
        self.conditional_object = conditional_object

    def tick(self):
        self.conditional_object.do_action(self.decorated)
        for node in self.decorated.tick():
            yield node
        if self.decorated.status != py_trees.common.Status.RUNNING and \
           self.decorated.status != py_trees.common.Status.SUCCESS:
                self.decorated.stop(self.decorated.status)
                self.terminate(self.decorated.status)
        self.status = self.decorated.status
        yield self

