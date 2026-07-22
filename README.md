Code of Master Thesis "Active object search in unknown open environments"

https://repository.tugraz.at/publications/tyqw1-vhp41

## Abstract

Everyday human environments have an underlying structure. Knowledge
about this structure is used by humans to carry out tasks efficiently. In this
thesis a robotic system is presented which uses such commonsense knowl-
edge to execute an intelligent active object search in unknown open indoor
environments. The main challenges are the selection of useful commonsense
knowledge, the design of a suitable representation of the environment, and
the development of a search planner which utilizes the gathered information.
The concept of rooms, and commonsense knowledge about the connection
between the room type of an area and the objects usually located in that
area are utilized in this thesis. Using the commonsense knowledge, the
room type of an area allows to estimate object probabilities in this area and
the room type detected after peeking into a room allows to estimate the
object probabilities in the whole room. A mapping system was developed
which creates a semantic map of the environment containing the room
structure and information about seen objects and room types of areas of the
environment. This information is extracted from images using convolutional
neural networks. Based on this map, a probabilistic planner generates view
poses for the robot to drive to which minimize the expected search time. Test
runs of the developed system showed a more intelligent behavior, where
likely areas were searched first, and an improved performance compared
to a coverage-maximizing search system. The findings of this work can be
used in similar tasks with the goal to create robotic systems being able to
help humans with a wide variety of tasks.
