from examples.framework import (Framework, Keys, main)
from Box2D import (b2CircleShape, b2EdgeShape, b2FixtureDef, b2PolygonShape,
                   b2Vec2, b2_pi)
import random
import numpy as np
import csv

class GP (Framework):
    name = "Genetic Programming Creatures"
    description = "Evolve creatures using genetic programming"
    offset = b2Vec2(0, 8)
    motorOn = True

    # Evolution parameters:
    max_simulation_time = 2.
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
        self.fitnesses = []
        self.average_complexity = 0
        self.complexities = []
        self.average_depth = 0
        self.depths = []

        self.fitness_file = 'fitness.csv'
        self.complexity_file = 'complexities.csv'
        self.depth_file = 'depths.csv'

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

        with open(self.fitness_file, 'w', newline='') as f:
            wr = csv.writer(f)
            wr.writerow(['generation', 'best fitness'])

        with open(self.complexity_file, 'w', newline='') as f:
            wr = csv.writer(f)
            wr.writerow(['generation', 'average complexity'])

        with open(self.depth_file, 'w', newline='') as f:
            wr = csv.writer(f)
            wr.writerow(['generation', 'average depth'])


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
            print('End of generation {}'.format(self.generation))
            self.best_fitness = max([creature.get_fitness() for creature in self.creatures])
            self.fitnesses.append(self.best_fitness)
            self.write_to_csv(self.fitness_file, self.best_fitness)
            print('Best fitness: {}'.format(self.best_fitness))
            # Calculate average creature complexity
            self.average_complexity = sum([len(creature.body)-1 for creature in self.creatures])/self.population_size
            self.complexities.append(self.average_complexity)
            self.write_to_csv(self.complexity_file, self.average_complexity)
            print('Average complexity: {}'.format(self.average_complexity))
            self.average_depth = sum([max([len(part.split("_")[-1]) for part in creature.graph.keys()]) for creature in self.creatures])/self.population_size
            self.depths.append(self.average_complexity)
            self.write_to_csv(self.depth_file, self.average_depth)
            print('Average depth: {}'.format(self.average_depth))

            self.generation += 1
            best_creatures = self.select_best_creatures(self.creatures, 5)
            best_graphs = [creature.graph for creature in best_creatures]
            new_graphs = self.evolve_creatures(best_graphs, self.population_size)
            new_population = [self.creature_from_graph(graph) for graph in new_graphs]
            self.destroy_creatures(self.creatures)
            self.creatures = new_population

            self.simulation_time = 0

    def write_to_csv(self, file, value):
        with open(file, 'a', newline='') as f:
            wr = csv.writer(f)
            wr.writerow([self.generation, value])


    def select_best_creatures(self, creatures, n=2):
        # Randomly select n best creatures based on fitness
        best_creatures = []
        fitnesses = [creature.get_fitness() for creature in creatures]
        max = sum(fitnesses)
        rel_fitness = [f / max for f in fitnesses]
        probs = [sum(rel_fitness[:i + 1]) for i in range(len(rel_fitness))]
        while len(best_creatures) < n:
            r = np.random.random()
            for (i, creature) in enumerate(creatures):
                if r <= probs[i]:
                    if creature not in best_creatures:
                        best_creatures.append(creature)
                    break
        return best_creatures

    def find_parent(self, g, tbmut):
        # find parent of tbmut in graph g
        for k, v in g.items():
            if tbmut in v:
                return k

    def delete_connections(self, g, tbmut, tbdel):
        # delete connection to child nodes
        for i in tbdel:
            del (g[i])
        # delete connection to parent node
        return {k: [vi for vi in v if vi != tbmut] for k, v in g.items()}


    def find_child(self, g, tbmut, tbdel):
        # find child nodes recursively
        tbdel.append(tbmut)
        for m in g[tbmut]:
            self.find_child(g, m, tbdel)


    def evolve_creatures(self, graphs, n=10, p_mut_part = 1.0, p_mut_param = 0.5, p_crossover = 0.5):
        # copy graphs until there are n
        new_graphs = []
        for i in range(n):
            new_graphs.append(graphs[i % len(graphs)].copy())
        # mutate every graph randomly
        for i, g in enumerate(new_graphs):
            if np.random.random() < p_mut_part:
                print('graph before: {}'.format(g))
                tbmut = random.choice([x for x in list(g.keys()) if not x.startswith('R')])
                print('to be mutated: {}'.format(tbmut))
                tbdel = []
                self.find_child(g, tbmut, tbdel)
                #print('to be deleted: {}'.format(tbdel))
                parent = self.find_parent(g, tbmut)
                print('parent: {}'.format(parent))
                g = self.delete_connections(g, tbmut, tbdel)
                #print('graph after: {}'.format(g))
                g = self.create_random_subgraph(parent, g)
                #print('mutated graph: {}'.format(g))
                new_graphs[i] = g

            elif np.random.random() < p_mut_param:
                tbmut = random.choice([x for x in list(g.keys()) if not x.startswith('R')])
                split = tbmut.split('_')
                part = self.decode_part(tbmut)
                angle = (part[1] + int(np.random.normal(scale=90))) % 360
                speed = np.clip([part[2] + int(np.random.normal(scale=5))], self.min_speed, self.max_speed)[0]
                length = np.clip([part[3] + int(np.random.normal(scale=2))], self.min_length, self.max_length)[0]
                new_tbmut = self.encode_part(split[0], angle, speed, length, split[-1])
                g[new_tbmut] = g.pop(tbmut)
                for k, v in g.items():
                    g[k] = [new_tbmut if i == tbmut else i for i in v]

            elif np.random.random() < p_crossover:
                og = g.copy()
                tbcross1 = random.choice([x for x in list(g.keys()) if not x.startswith('R')])
                children1 = []
                self.find_child(g, tbcross1, children1)
                subgraph1 = {c: g[c] for c in children1}
                parent1 = self.find_parent(g, tbcross1)
                g = self.delete_connections(g, tbcross1, children1)
                # Do not mutate with itself (or its clone)
                j, g2 = random.choice([(j, g2) for j, g2 in enumerate(new_graphs) if j % len(graphs) is not i % len(graphs)])
                og2 = g2.copy()
                tbcross2 = random.choice([x for x in list(g2.keys()) if not x.startswith('R')])
                children2 = []
                self.find_child(g2, tbcross2, children2)
                subgraph2 = {c: g2[c] for c in children2}
                parent2 = self.find_parent(g2, tbcross2)
                g2 = self.delete_connections(g2, tbcross2, children2)
                # Add subgraphs
                self.add_subgraph(g, parent1, tbcross2, subgraph2)
                self.add_subgraph(g2, parent2, tbcross1, subgraph1)
                if max([len(part.split("_")[-1])-1 for part in g.keys()]) < self.max_depth and max([len(part.split("_")[-1])-1 for part in g2.keys()])  < self.max_depth:
                    new_graphs[i] = g
                    new_graphs[j] = g2
                else:
                    new_graphs[i] = og
                    new_graphs[j] = og2

        return new_graphs


    def add_subgraph(self, graph, parent, root, subgraph):
        # Lots of code to fix the graphcode
        parentcode = parent.split("_")[-1]
        rootcode = root.split("_")[-1]
        kids = graph[parent]
        missing_nr = 0
        if len(kids) > 0:
            missing_nr = sorted(set(range(len(kids)+1)).difference([int(x.split("_")[-1][-1]) for x in kids]))[0]
        newcode = parentcode + str(missing_nr)
        kids.append(self.new_graph_code(root, rootcode, newcode))
        for k, v in subgraph.items():
            new_k = self.new_graph_code(k, rootcode, newcode)
            new_v = [self.new_graph_code(c, rootcode, newcode) for c in v]
            graph[new_k] = new_v


    def new_graph_code(self, part, oldroot, newroot):
        split = part.split("_")
        newcode = split[-1].replace(oldroot, newroot, 1)
        newpart = "_".join(split[:-1] + [newcode])
        return newpart


    def create_random_subgraph(self, root, graph):
        # Continue mutated graph from the parent of the deleted node downward

        anchor_set = {'L': 1, 'E': 0, 'S': 2, 'R': 1}
        openlist = [root] # add parent of tbmut to openlist as root node

        while len(openlist) > 0:
            part = openlist.pop()
            anchors = anchor_set[part[0]]
            part_code = part.split('_')[-1]
            graph[part] = []

            for i in range(anchors):

                if len(part_code) < self.max_depth:
                    name, _ = random.choice(list(self.primitive_set.items()))
                else:
                    name = random.choice(self.terminal_set)

                prefix = name
                angle = np.random.randint(0, 360)
                speed = np.random.randint(self.min_speed, self.max_speed)
                length = np.random.randint(self.min_length, self.max_length)
                graph_code = part_code + str(i)

                openlist.append(self.encode_part(prefix, angle, speed, length, graph_code))
                graph[part].append(self.encode_part(prefix, angle, speed, length, graph_code))

        return graph

    def creature_from_graph(self, graph):
        root = self.create_root()
        openlist = [root]
        partlist = [root[0]]

        while len(openlist) > 0:
            body, anchors, enc = openlist.pop()

            for i, anchor in enumerate(anchors):
                next_enc = graph[enc][i]
                func, angle, speed, length, graph_code = self.decode_part(next_enc)
                # create part and anchors
                part = func(anchor,
                            body,
                            graph_code,
                            angle=angle,
                            speed=speed,
                            length=length)
                if len(part[1]) > 0:
                    openlist.append(part)
                partlist.append(part[0])

        return Creature(partlist, graph, self.generation)


    def destroy_creatures(self, creatures):
        for creature in creatures:
            for body in creature.body:
                self.world.DestroyBody(body)


    def encode_part(self, prefix, angle, speed, length, graph_code):
        return '{}_{:03d}_{:03d}_{:03d}_{}'.format(prefix, angle, speed, length, graph_code)


    def decode_part(self, str):
        split = str.split('_')
        return self.primitive_set[split[0]], int(split[1]), int(split[2]), int(split[3]), split[4]


    def create_root(self, not_create=False):
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

        if not_create:
            self.world.DestroyBody(body)

        return body, anchors, 'R_0'


    def create_arm(self, anchor, connected, graph_code, angle=45, speed=5, length=4, not_create=False):

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

        if not_create:
            self.world.DestroyBody(body)

        return body, anchors, self.encode_part('L', angle, speed, length, graph_code)


    def create_split(self, anchor, connected, graph_code, angle=45, speed=5, length=4, not_create=False):

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

        if not_create:
            self.world.DestroyBody(body)

        return body, anchors, self.encode_part('S', angle, speed, length, graph_code)


    def create_endpoint(self, anchor, connected, graph_code, angle=0, speed=5, length=2, not_create=False):
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

        if not_create:
            self.world.DestroyBody(body)

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
                openlist.append(part)
                graph[enc].append(part[2])
                partlist.append(part[0])
        return Creature(partlist, graph, self.generation)


class Creature:

    def __init__(self, body, graph, generation):
        self.body = body
        self.graph = graph
        self.time_alive = 0  # time in seconds
        self.generation = generation

    def get_fitness(self):
        # return x coordinate of root as fitness value
        return max(self.body[0].position.x, 0)

    def update(self, timestep):
        self.time_alive += timestep
        # do this every simulation step (such as checking and updating time_alive)
        pass


if __name__ == "__main__":
    main(GP)
