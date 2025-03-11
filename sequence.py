#!/usr/bin/env python3


import time
import py_trees
import random

class BTAction(py_trees.behaviour.Behaviour):


    def __init__(self, name: str = "BTAction"):

        """Configure the name of the behaviour."""
        super(BTAction, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))


    def setup(self, **kwargs: int) -> None:
        """No delayed initialisation required for this example."""
        self.logger.debug("%s.setup()" % (self.__class__.__name__))


    def initialise(self) -> None:
        """Reset a BTAction variable."""
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))
        self.BTAction = 0


    def update(self) -> py_trees.common.Status:
       
        random_integer = random.randrange(1, 10)
        if( random_integer > 5 ):
            self.feedback_message = (
                "Number {0} is greater than 5: Failure ".format(random_integer)
            )
            new_status = py_trees.common.Status.FAILURE
        else:
            self.feedback_message = (
                "Number {0} is less than 5: Success ".format(random_integer)
            )            
            new_status = py_trees.common.Status.SUCCESS
        
        return new_status


def create_root() -> py_trees.behaviour.Behaviour:

    root = py_trees.composites.Sequence(name="Sequence", memory=False)

    for action in ["Action 1", "Action 2", "Action 3"]:
        node = BTAction(name=action)
        root.add_child(node)
    return root



def main() -> None:

    py_trees.logging.level = py_trees.logging.Level.DEBUG

    root = create_root()
    root.setup_with_descendants()

    for i in range(1, 6):

        try:
            print("\n--------- Tick {0} ---------\n".format(i))
            root.tick_once()
            print("\n")
            print(py_trees.display.unicode_tree(root=root, show_status=True))
            time.sleep(1.0)
        except KeyboardInterrupt:
            break
    print("\n")
    
if __name__ == '__main__':
    main()