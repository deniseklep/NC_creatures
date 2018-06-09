from examples.framework import (Framework, Keys, main)
from Box2D import (b2CircleShape, b2EdgeShape, b2FixtureDef, b2PolygonShape,
                   b2Vec2, b2_pi)
import random


class GP (Framework):
    name = "Genetic Programming Creatures"
    description = "Evolve creatures using genetic programming"
    offset = b2Vec2(0, 8)
    motorOn = True

    def __init__(self):
        super(GP, self).__init__()

        self.function_set = {'leg': self.create_arm,
                             'end': self.create_endpoint,}

        # The ground
        ground = self.world.CreateStaticBody(
            shapes=[
                b2EdgeShape(vertices=[(-1000, 0), (1000, 0)]),
            ]
        )

        box = b2FixtureDef(
            shape=b2PolygonShape(box=(0.5, 0.5)),
            density=1,
            friction=0.3)
        circle = b2FixtureDef(
            shape=b2CircleShape(radius=0.25),
            density=1)


    def create_root(self):
        body = self.world.CreateDynamicBody(
            position=self.offset,
            fixtures=b2FixtureDef(
                shape=b2CircleShape(radius=1),
                density=1),
        )

        # define joints
        anchors = []
        anchors.append(self.offset)

        return body, anchors


    def create_arm(self, anchor, connected, angle=0, speed=0, length=2):

        #define body
        p1 = b2Vec2(0, -1)
        p2 = b2Vec2(-1, 0)
        p3 = b2Vec2(-1, length)
        p4 = b2Vec2(0, length+2)
        p5 = b2Vec2(1, length)
        p6 = b2Vec2(1, 0)

        poly = b2PolygonShape(vertices=(p1, p2, p3, p4, p5, p6))

        body = self.world.CreateDynamicBody(
            position=anchor,
            angle=angle,
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

        return body, anchors


    def create_endpoint(self, anchor, connected, angle=0, speed=0, length=2):
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
            angle=angle,
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

        return body, anchors


    def create_prototype(self):
        root = self.create_root()
        openlist = [root]
        graph = {}

        while len(openlist) > 0:
            body, anchors = openlist.pop()
            connected = []
            for anchor in anchors:
                name, func = random.choice(list(self.function_set.items()))
                part = func()
                connected.append()




if __name__ == "__main__":
    main(GP)
