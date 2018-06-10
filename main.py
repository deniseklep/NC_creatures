from examples.framework import (Framework, Keys, main)
from Box2D import (b2CircleShape, b2EdgeShape, b2FixtureDef, b2PolygonShape,
                   b2Vec2, b2_pi)
import random
import numpy as np


class GP (Framework):
    name = "Genetic Programming Creatures"
    description = "Evolve creatures using genetic programming"
    offset = b2Vec2(0, 8)
    motorOn = True

    def __init__(self):
        super(GP, self).__init__()

        self.function_set = {'L': self.create_arm,
                             'E': self.create_endpoint,}

        self.actors = []

        # The ground
        ground = self.world.CreateStaticBody(
            shapes=[
                b2EdgeShape(vertices=[(-1000, 0), (1000, 0)]),
            ]
        )

        obstacle_count = 400

        # Pre-defined obstacle types
        box = b2FixtureDef(
            shape=b2PolygonShape(box=(0.6, 0.6)),
            density=1,
            friction=0.3)
        circle = b2FixtureDef(
            shape=b2CircleShape(radius=0.2),
            density=1)
        triangle = b2FixtureDef(
            shape=b2PolygonShape(vertices=[(0,0),(1,1),(0,1)]),
            density=1)

        # Randomly generated obstacle shapes and distances
        obstacles = [box, circle, triangle]
        for i in range(obstacle_count):
            obstacle = self.world.CreateDynamicBody(
                fixtures=random.choice(obstacles),
                position=(-40 + np.int(np.random.choice(40, 1).squeeze()) * i, 0.5))

        self.actors.append(self.create_prototype())


    # Executed at every simulation step, put GP logic here:
    def Step(self, settings):
        super(GP, self).Step(settings)

        for actor in self.actors:
            actor.update(1.0 / settings.hz)


    def encode_part(self, prefix, angle, speed, length):
        return '{}_{:03d}_{:03d}_{:03d}'.format(prefix, angle, speed, length)


    def decode_part(self, str):
        split = str.split('_')
        return self.function_set[split[0]], int(split[1]), int(split[2]), int(split[3])


    def create_root(self):
        body = self.world.CreateDynamicBody(
            position=self.offset,
            fixtures=b2FixtureDef(
                shape=b2CircleShape(radius=0.1),
                density=1),
        )

        # define joints
        anchors = []
        anchors.append(self.offset)
        anchors.append(self.offset)

        return body, anchors, 'R'


    def create_arm(self, anchor, connected, angle=0, speed=2, length=4):

        #define body
        p1 = b2Vec2(0, -1)
        p2 = b2Vec2(-1, 0)
        p3 = b2Vec2(-1, length)
        p4 = b2Vec2(0, length+1)
        p5 = b2Vec2(1, length)
        p6 = b2Vec2(1, 0)

        poly = b2PolygonShape(vertices=(p1, p2, p3, p4, p5, p6))

        body = self.world.CreateDynamicBody(
            position=anchor,
            angle=angle * b2_pi / 180,
            angularDamping=10,
            fixtures=b2FixtureDef(
                shape=poly,
                groupIndex=-1,
                density=1),
        )

        #define joints
        anchors = []
        anchors.append(b2Vec2(anchor.x, anchor.y+length))

        motorJoint = self.world.CreateRevoluteJoint(
            bodyA=body,
            bodyB=connected,
            anchor=anchor,
            collideConnected=False,
            motorSpeed=speed,
            maxMotorTorque=1000,
            enableMotor=self.motorOn)

        return body, anchors, self.encode_part('L', angle, speed, length)


    def create_endpoint(self, anchor, connected, angle=0, speed=2, length=2):
        # define body
        p1 = b2Vec2(0, -1)
        p2 = b2Vec2(-1, 0)
        p3 = b2Vec2(-1, length)
        p4 = b2Vec2(0, length+2)
        p5 = b2Vec2(1, length)
        p6 = b2Vec2(1, 0)

        poly = b2PolygonShape(vertices=(p1, p2, p3, p4, p5, p6))

        body = self.world.CreateDynamicBody(
            position=anchor,
            angle=angle * b2_pi / 180,
            angularDamping=10,
            fixtures=b2FixtureDef(
                shape=poly,
                groupIndex=-1,
                density=1),
        )

        # define joints
        anchors = []

        motorJoint = self.world.CreateRevoluteJoint(
            bodyA=body,
            bodyB=connected,
            anchor=anchor,
            collideConnected=False,
            motorSpeed=speed,
            maxMotorTorque=1000,
            enableMotor=self.motorOn)

        return body, anchors, self.encode_part('E', angle, speed, length)


    def create_prototype(self):
        root = self.create_root()
        openlist = [root]
        partlist = [root[0]]
        graph = {root[2]: []}

        while len(openlist) > 0:
            body, anchors, enc = openlist.pop()
            graph[enc] = []

            for anchor in anchors:
                # choose random part
                name, func = random.choice(list(self.function_set.items()))
                # create part and anchors
                part = func(anchor, body)
                #
                openlist.append(part)
                graph[enc].append(part[2])
                partlist.append(part[0])

        return Creature(partlist, graph)


class Creature:

    def __init__(self, body, graph, generation):
        self.body = body
        self.graph = graph
        self.time_alive = 0
        self.generation = generation

    def get_fitness(self):
        # return x coordinate of root as fitness value
        return self.body[0].position.x

    def update(self, timestep):
        self.time_alive += timestep
        # do this every simulation step (such as checking and updating time_alive)
        pass


if __name__ == "__main__":
    main(GP)
