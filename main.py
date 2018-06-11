from examples.framework import (Framework, Keys, main)
from Box2D import (b2CircleShape, b2EdgeShape, b2FixtureDef, b2PolygonShape,
                   b2Vec2, b2_pi)
import random
import numpy as np
from random import shuffle


class GP (Framework):
    name = "Genetic Programming Creatures"
    description = "Evolve creatures using genetic programming"
    offset = b2Vec2(0, 8)
    motorOn = True

    # Evolution parameters:
    max_simulation_time = 30.
    population_size = 20
    max_depth = 5
    terminal_set = ['E']
    function_set = ['L', 'S']

    #part ranges
    min_speed = -10
    max_speed = 10
    min_length = 1
    max_length = 5

    def __init__(self):
        super(GP, self).__init__()

        self.primitive_set = {'L': self.create_arm,
                              'E': self.create_endpoint,
                              'S': self.create_split}

        self.creatures = []
        self.simulation_time = 0
        self.generation = 0
        self.best_fitness = 0

        # The ground
        ground = self.world.CreateStaticBody(
            shapes=[
                b2EdgeShape(vertices=[(-1000, 0), (1000, 0)]),
            ]
        )

        obstacle_count = 400

        # Pre-defined obstacle types
        box = b2FixtureDef(
            shape=b2PolygonShape(box=(0.6, 0.6)), #large
            density=1,
            friction=0.3)
        circle = b2FixtureDef(
            shape=b2CircleShape(radius=0.2), #small
            density=1)
        triangle = b2FixtureDef(
            shape=b2PolygonShape(vertices=[(0,0),(1,1),(0,1)]), #middle
            density=1)

        # Randomly generated obstacle shapes and distances
        obstacles = [box, circle, triangle]
        for i in range(obstacle_count):
            obstacle = self.world.CreateStaticBody(
                fixtures=random.choice(obstacles),
                position=(-40 + np.int(np.random.choice(40, 1).squeeze()) * i, 0.5))

        # Create initial population
        for i in range(self.population_size):
            self.creatures.append(self.create_prototype())


    def Step(self, settings):
        super(GP, self).Step(settings)

        # Executed at every simulation step, put GP logic here:
        self.simulation_time += 1.0 / settings.hz
        if self.simulation_time < self.max_simulation_time:
            # Update all creatures
            for creature in self.creatures:
                creature.update(1.0 / settings.hz)
        else:
            # Create new generation
            self.simulation_time = 0
            self.generation += 1
            best_creatures = self.select_best_creatures(self.creatures)
            best_graphs = [creature.graph for creature in best_creatures]
            #new_graphs = self.evolve_creatures(best_graphs, self.population_size)
            #new_population = [self.creature_from_graph(graph) for graph in new_graphs]
            # TODO: delete old creatures
            #self.creatures = new_population

            print('Best fitness: {}'.format(self.best_fitness))


    def select_best_creatures(self, creatures, n=2):
        # Randomly select n best creatures based on fitness
        best_creatures = []
        max = sum([creature.get_fitness() for creature in creatures])
        while len(best_creatures) < n:
            select = random.uniform(0,max)
            current = 0
            for creature in creatures:
                current += creature.get_fitness()
                if current > select:
                    best_creatures.append(creature)
        shuffle(best_creatures)
        delete = len(best_creatures)-n
        del best_creatures[-delete:]
        return best_creatures


    def evolve_creatures(self, graphs, n=10):
        # TODO: create n new creatures by random mutation and crossover of the graphs

        return graphs


    def creature_from_graph(self, graph):
        # TODO: create new creature parts from graph
        partlist = [self.create_root()[0]]

        return Creature(partlist, graph, self.generation)


    def encode_part(self, prefix, angle, speed, length, graph_code):
        return '{}_{:03d}_{:03d}_{:03d}_{}'.format(prefix, angle, speed, length, graph_code)


    def decode_part(self, str):
        split = str.split('_')
        return self.primitive_set[split[0]], int(split[1]), int(split[2]), int(split[3]), split[4]


    def create_root(self):
        body = self.world.CreateDynamicBody(
            position=self.offset,
            fixtures=b2FixtureDef(
                shape=b2CircleShape(radius=0.1),
                groupIndex=-1,
                density=1),
        )

        # define joints
        anchors = []
        anchors.append(self.offset)

        return body, anchors, 'R_0'


    def create_arm(self, anchor, connected, graph_code, angle=45, speed=5, length=4):

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
        rad1 = (angle + 90) * b2_pi / 180
        a1 = b2Vec2(anchor.x + np.cos(rad1) * length, anchor.y + np.sin(rad1) * length)
        anchors = [a1]

        motorJoint = self.world.CreateRevoluteJoint(
            bodyA=body,
            bodyB=connected,
            anchor=anchor,
            collideConnected=False,
            motorSpeed=speed,
            maxMotorTorque=1000,
            enableMotor=self.motorOn)

        return body, anchors, self.encode_part('L', angle, speed, length, graph_code)


    def create_split(self, anchor, connected, graph_code, angle=45, speed=5, length=4):

        #define body
        p1 = b2Vec2(0, -1)
        p2 = b2Vec2(-1, 0)
        p3 = b2Vec2(-2, length+1)
        p4 = b2Vec2(0, length+2)
        p5 = b2Vec2(2, length+1)
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
        rad1 = (angle+80) * b2_pi / 180
        rad2 = (angle+100) * b2_pi / 180
        a1 = b2Vec2(anchor.x + np.cos(rad1) * length, anchor.y + np.sin(rad1) * length)
        a2 = b2Vec2(anchor.x + np.cos(rad2) * length, anchor.y + np.sin(rad2) * length)
        anchors = [a1, a2]

        motorJoint = self.world.CreateRevoluteJoint(
            bodyA=body,
            bodyB=connected,
            anchor=anchor,
            collideConnected=False,
            motorSpeed=speed,
            maxMotorTorque=1000,
            enableMotor=self.motorOn)

        return body, anchors, self.encode_part('S', angle, speed, length, graph_code)


    def create_endpoint(self, anchor, connected, graph_code, angle=0, speed=5, length=2):
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

        return body, anchors, self.encode_part('E', angle, speed, length, graph_code)


    def create_prototype(self):
        root = self.create_root()
        openlist = [root]
        partlist = [root[0]]
        graph = {root[2]: []}

        while len(openlist) > 0:
            body, anchors, enc = openlist.pop()
            graph_code = enc.split('_')[-1]
            graph[enc] = []

            for i, anchor in enumerate(anchors):
                # choose random part
                if len(graph_code) < self.max_depth:
                    name, func = random.choice(list(self.primitive_set.items()))
                else:
                    name = random.choice(self.terminal_set)
                    func = self.primitive_set[name]
                # create part and anchors
                part = func(anchor,
                            body,
                            graph_code+str(i),
                            angle=np.random.randint(0, 360),
                            speed=np.random.randint(self.min_speed, self.max_speed),
                            length=np.random.randint(self.min_length, self.max_length))
                # add unique graph string to encoding
                if len(part[1]) > 0:
                    openlist.append(part)
                graph[enc].append(part[2])
                partlist.append(part[0])
        print(graph)

        return Creature(partlist, graph, self.generation)


class Creature:

    def __init__(self, body, graph, generation):
        self.body = body
        self.graph = graph
        self.time_alive = 0  # time in seconds
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
