import sumolib
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import matplotlib.animation as animation
import sys
import copy
import os
import csv
import random

a = 202
#a = int(sys.argv[1])
print("seed値 : " + str(a))
np.random.seed(a)

# Read the network
#network = sumolib.net.readNet("grid3x3.net.xml")
#road_filename = "3x3" 
network = sumolib.net.readNet("grid5x5.net.xml")
road_filename = "5x5" 
G = nx.DiGraph()

num_cars = 600
num_obstacles = 2
num_fakecar = 3
num_fakeobstacles = 1

Field_of_View = 30
Max_Speed = 3

fig, ax = plt.subplots()
dots, = ax.plot([], [], 'go', markersize=5)
fdots, = ax.plot([], [], "mo", markersize = 5)
obstacles, = ax.plot([], [], 'rD', markersize=3)
fobstacles, = ax.plot([], [], "cD", markersize = 3)
ax.axis('off')

cars_list = []
fakecar_list = []
allcar_list = []
obstacle_list = []
obstacle_positions = []
fake_obstacle_positions = []
used_edges = []
total_obstacle_list = []
fake_obstacle_list = []

node_list = []
edge_list = []
car_edge = {}

node_queues = {}

arrival_times = []

total_distances_list = []


class Car:
    def __init__(self, G, i):
        self.G = copy.deepcopy(G)
        self.number = i
        self.encountered_obstacles = []
        self.encountered_obstacles_edges = []
        self.fake_edge_list = []
        self.current_node = np.random.choice(list(self.G.nodes()))
        self.target_node = np.random.choice(list(self.G.nodes()))

        if self.current_node not in node_queues:
            node_queues[self.current_node] = []
        node_queues[self.current_node].append(self)

        #while self.current_node == self.target_node:
            #self.target_node = np.random.choice(list(self.G.nodes()))

        self.path = nx.astar_path(self.G, self.current_node, self.target_node, heuristic=None)

        while self.current_node == self.target_node or len(self.path) < 3:
            self.target_node = np.random.choice(list(self.G.nodes()))
            self.path = nx.astar_path(self.G, self.current_node, self.target_node, heuristic=None)

        self.current_pos = np.array(self.G.nodes[self.current_node]['pos'])
        self.next_node_index = 1
        self.next_node = self.path[1] if len(self.path) > 1 else None
        self.next_node_pos = np.array(self.G.nodes[self.next_node]['pos'])
        self.current_edge = (self.current_node, self.next_node) if self.next_node else None
        car_edge[self.current_edge].append(self)
        self.target_pos = np.array(self.G.nodes[self.next_node]['pos']) if self.next_node else self.current_pos
        self.max_speed = Max_Speed
        self.speed = 0
        self.inter_car_distance = 0
        self.field_of_view = Field_of_View
        self.nearest = 0
        self.stop = 0
        self.brake = 0
        self.ready_to_move = False 
        

        self.total_distance_traveled = 0


    def walk(self):
        #print(self.speed, self.inter_car_distance,self.nearest, self.stop, self.brake)
        if self.ready_to_move: 
            current_queue = node_queues.get(self.current_node, [None])
            if current_queue and current_queue[0] == self:
                node_queues[self.current_node].pop(0)

            direction = self.target_pos - self.current_pos
            norm = np.linalg.norm(direction)

            if self in car_edge[self.current_edge]:
                car_edge[self.current_edge].remove(self)

            self.inter_car_distance = self.field_of_view
            if self.brake == 0:
                self.nearest = self.inter_car_distance
            self.stop = 0

            self.brake = 0

            for car in car_edge[self.current_edge]:
                for edge in self.encountered_obstacles_edges:
                    if edge[:2] == self.current_edge:
                        self.nearest = self.nearest/2
                        self.brake = 1
                if(np.linalg.norm(self.current_pos - self.next_node_pos) > np.linalg.norm(car.current_pos - car.next_node_pos)):
                    if(self.nearest > np.linalg.norm(self.current_pos - car.current_pos)):
                        if (np.linalg.norm(self.current_pos - car.current_pos)==0):
                            if (self.number > car.number):
                                self.nearest = np.linalg.norm(self.current_pos - car.current_pos)
                        elif(np.linalg.norm(self.current_pos - car.current_pos)<=1):
                            if self.number < car.number:
                                pass
                            else:
                                self.stop = 1
                        else:
                            self.nearest = np.linalg.norm(self.current_pos - car.current_pos)

            for obstacle in obstacle_list:
                if obstacle.edge[:2] == self.current_edge:
                    if(self.nearest > np.linalg.norm(self.current_pos - obstacle.position)):
                        self.nearest = np.linalg.norm(self.current_pos - obstacle.position)
                

            self.speed = 0.5*self.max_speed*(np.tanh(self.nearest/5 - 2) + np.tanh(2))
            if self.stop == 1:
                self.speed = 0

            if(self.speed > self.nearest):
                self.speed = self.nearest

            if norm == 0:
                step = 0

            if norm > 0:
                step = self.speed * direction / norm
                self.current_pos += step

            self.exchange_info_with_nearby_cars(car_edge[(self.current_node),(self.next_node)])
            self.exchange_info_with_nearby_cars(car_edge[(self.next_node),(self.current_node)])

            for obstacle in total_obstacle_list:
                if obstacle.edge[:2] == self.current_edge:
                    if (np.linalg.norm(obstacle.position - self.current_pos) < self.field_of_view and self.speed <= 1 and type(obstacle)is Obstacle):
                        self.encountered_obstacles_edges.append(self.current_edge)
                        self.Uturn()
                    else:
                        for edge in self.encountered_obstacles_edges:
                            if (edge[:2] == self.current_edge and self.speed <= 0.1):
                                self.Uturn()
                    if np.linalg.norm(obstacle.position - self.current_pos) < self.speed:
                        self.speed=self.speed/2

            # 移動処理
            if np.linalg.norm(self.target_pos - self.current_pos) < self.speed and not any(tuple(self.current_edge) == tuple(obstacle_edge[:2]) for obstacle_edge in obstacle_positions):
                self.current_pos = self.target_pos
                self.current_node = self.next_node
            # 新しい経路の計算
                if self.current_node != self.target_node:
                    self.path_recalculation()
                else:
                    return True

            car_edge[self.current_edge].append(self)

            distance_moved = np.linalg.norm(step)
            self.total_distance_traveled += distance_moved

            return False

    
    def update_current_edge(self):
        if self.next_node_index < len(self.path):
            self.current_edge = (self.path[self.next_node_index - 1], self.path[self.next_node_index])
            
        else:
            self.current_edge = None


    def exchange_info_with_nearby_cars(self, current_edge_list):
        for other_car in current_edge_list:
            if other_car.number > self.number:
                continue  # インデックスが自分より大きいポイントは無視

            distance = np.linalg.norm(self.current_pos - other_car.current_pos)
            if distance <= 5 and other_car.ready_to_move == True:  # ノルムが5以下の場合
                # 障害物情報の交換
                    self.encountered_obstacles_edges = list(set(self.encountered_obstacles_edges + other_car.encountered_obstacles_edges + other_car.fake_edge_list) - set(self.fake_edge_list))
                    other_car.encountered_obstacles_edges = list(set(other_car.encountered_obstacles_edges + self.encountered_obstacles_edges + self.fake_edge_list) - set(other_car.fake_edge_list))

    def path_recalculation(self):
        for edge in self.encountered_obstacles_edges:
            if self.G.has_edge(*edge):
                self.G.remove_edge(*edge)
        self.path = nx.astar_path(self.G, self.current_node, self.target_node, heuristic=None)
        self.next_node_index = 1
        self.next_node = self.path[self.next_node_index] if len(self.path) > 1 else None
        self.target_pos = np.array(self.G.nodes[self.next_node]['pos']) if self.next_node else self.current_pos
        if self in car_edge[self.current_edge]:
            car_edge[self.current_edge].remove(self)
        self.current_edge = ((self.current_node),(self.next_node))

    def Uturn(self):
        if self.G.has_edge(self.current_edge[1], self.current_edge[0]):
                # Uターンを開始
            self.current_node, self.next_node = self.next_node, self.current_node  # ノードを交換
            self.current_edge = (self.current_edge[1], self.current_edge[0])  # エッジを反転
            print(self.current_edge)
            self.target_pos = np.array(self.G.nodes[self.next_node]['pos'])
            self.brake = 0
    
    def get_total_distance_traveled(self):
        return self.total_distance_traveled
    
class FakeCar(Car):
    def __init__(self, G, i):
        super().__init__(G, i)
        self.fake_obstacle_list = []


    def create_fake_obstacles(self, num_fakeobstacles):
        global total_obstacle_list, used_edges
        self.path_edges = [(self.path[i], self.path[i+1]) for i in range(len(self.path)-1)]
        all_edges = list(self.G.edges(data=True))

        for _ in range(num_fakeobstacles):
            while True:
                # パス上のエッジからランダムに選択
                random_edge = random.choice(self.path_edges)
                edge_data = self.G.get_edge_data(*random_edge)

                # すでに使われているエッジでないことを確認
                if not any(random_edge == edge[:2] for edge in used_edges):
                    obstacle = FakeObstacle((random_edge[0], random_edge[1], edge_data))
                    total_obstacle_list.append(obstacle)
                    self.fake_obstacle_list.append(obstacle)
                    self.fake_edge_list.append(random_edge)
                    used_edges.append((random_edge[0], random_edge[1], edge_data))
                    used_edges.append((random_edge[1], random_edge[0], edge_data))
                    fake_obstacle_positions.append(edge_data['pos'])
                    break

        #for _ in range(num_fakeobstacles):
            #while True:
                #random_index = np.random.randint(0, len(all_edges))
                #random_edge = all_edges[random_index]
                #if self.current_node not in random_edge[:2] and not any(random_edge[:2] == edge[:2] for edge in used_edges):
                    #obstacle = FakeObstacle(random_edge)
                    #total_obstacle_list.append(obstacle)
                    #self.fake_obstacle_list.append(obstacle)
                    #self.fake_edge_list.append(random_edge[:2])
                    #used_edges.append(random_edge)
                    #used_edges.append((random_edge[1], random_edge[0],["pos"]))
                    #fake_obstacle_positions.append(random_edge[2]['pos'])
                    #break




class Obstacle:
    def __init__(self, edge):
        self.edge = edge
        self.position = edge[2]['pos'] 

class FakeObstacle(Obstacle):
    def __init__(self, edge):
        super().__init__(edge)


# Create the network graph
for node in network.getNodes():
    node_pos = node.getCoord()
    node_id = node.getID()
    node_list.append({'id': node_id, 'pos': (node_pos[0], node_pos[1])})
    G.add_node(node_id, pos=(node_pos[0], node_pos[1]))

for edge in network.getEdges():
    src_node = edge.getFromNode()
    dest_node = edge.getToNode()
    src_id = src_node.getID()
    dest_id = dest_node.getID()
    edge_id = edge.getID()
    edge_list.append({'id': edge_id, 'src': src_id, 'dest': dest_id})
    src_pos = np.array(src_node.getCoord())
    dest_pos = np.array(dest_node.getCoord())
    G.add_edge(src_id, dest_id, pos=dest_pos - 0.01 * (dest_pos - src_pos))

car_edge = {(edge["src"], edge["dest"]): [] for edge in edge_list}

# Create cars
for i in range(num_cars):
    car = Car(G,i)
    cars_list.append(car)
    allcar_list.append(car)

for i in range(num_fakecar):
    fakecar = FakeCar(G,i)
    fakecar_list.append(fakecar)
    allcar_list.append(fakecar)

for _ in fakecar_list:
    _.create_fake_obstacles(num_fakeobstacles)

print(used_edges)

# Place obstacles
all_edges = list(G.edges(data=True))
for _ in range(num_obstacles):
    while True:
        random_index = np.random.randint(0, len(all_edges))
        random_edge = all_edges[random_index]
        print(random_edge, used_edges)
        #if random_edge not in used_edges:
        if not any(random_edge[:2] == edge[:2] for edge in used_edges):
            obstacle = Obstacle(random_edge)
            total_obstacle_list.append(obstacle)
            obstacle_list.append(obstacle)
            used_edges.append(random_edge)
            used_edges.append((random_edge[1], random_edge[0],["pos"]))
            obstacle_positions.append(random_edge[2]['pos'])
            break


#グラフの接続テスト
updated_G = G.copy()
for u, v, _ in used_edges:
    updated_G.remove_edge(u,v)
if nx.is_strongly_connected(updated_G) == True:  
    print("接続テスト完了")
    pass
else:
    print("到達できないnodeがあります")
    sys.exit()



def init():
    node_positions = nx.get_node_attributes(G, 'pos')
    nx.draw(G, node_positions, node_size=5, node_color='blue', ax=ax, arrowstyle=None, arrows=False)
    dots.set_data([], [])
    obstacles.set_data([], [])
    return dots, obstacles

def update(num):
    print(num)
    xdata = []
    ydata = []
    fxdata = []
    fydata = []
    
    for node, queue in node_queues.items():
        if queue:
            queue[0].ready_to_move = True

    cars_to_remove = []

    for car in allcar_list:
        reached = car.walk()

        if not reached:
            x, y = car.current_pos
            #if isinstance(car, Car):   この書式はサブクラスも巻き込む、FakeCarからやると可能
            if type(car) is Car:
                xdata.append(x)
                ydata.append(y)
            elif type(car) is FakeCar:
                fxdata.append(x)
                fydata.append(y)
        else:
            cars_to_remove.append(car)
            arrival_time = num  
            arrival_times.append(arrival_time)
            total_distance = car.get_total_distance_traveled()
            total_distances_list.append((total_distance))

    for car in cars_to_remove:
        allcar_list.remove(car)

    dots.set_data(xdata, ydata)
    fdots.set_data(fxdata, fydata)

    if len(allcar_list) == 0:
        print("All cars have reached their destinations.")

        plt.clf()
        # 総移動距離のヒストグラムを保存
        plt.hist(total_distances_list, bins=50, rwidth=0.9, color='b')  
        plt.xlabel("moving dictance")  
        plt.ylabel("num car")

        # 保存先の相対パス
        histogram_folder = "moving_distance_hist"
        histogram_filename = f"総移動距離({a}) {road_filename} {num_cars} {num_obstacles} fake_car{num_fakecar}seed{a}.png" #{num_fake_cars} １両あたりが保持する偽障害物({having_fake_obstacle})seed{a}.png"
        save_path_name = os.path.join(histogram_folder, histogram_filename)

        # ヒストグラムを保存
        plt.savefig(save_path_name)
        plt.clf()

        # ゴールタイムのヒストグラムを保存
        plt.hist(arrival_times, bins=50, rwidth=0.9, color='b')  
        plt.xlabel("goal time")  
        plt.ylabel("num car")

        # 保存先の相対パス
        histogram_folder2 = "goal_time_hist"
        histogram_filename2 = f"ゴールタイム({a}) {road_filename} {num_cars} {num_obstacles}fake_car{num_fakecar}seed{a}.png"   #{num_fake_cars} {having_fake_obstacle}seed{a}.png"
        save_path_name2 = os.path.join(histogram_folder2, histogram_filename2)

        # ヒストグラムを保存
        plt.savefig(save_path_name2)
        plt.clf()

        plt.close(fig)

        # CSVファイルに経過時間を書き込み保存
        save_to_csv("arrival_times.csv", arrival_times, total_distances_list)  # total_distances_list を追加

    obstacle_xdata = [obs[0] for obs in obstacle_positions]
    obstacle_ydata = [obs[1] for obs in obstacle_positions]
    obstacles.set_data(obstacle_xdata, obstacle_ydata)

    obstacle_fxdata = [obs[0] for obs in fake_obstacle_positions]
    obstacle_fydata = [obs[1] for obs in fake_obstacle_positions]
    fobstacles.set_data(obstacle_fxdata, obstacle_fydata)


    return dots, fdots, obstacles, fobstacles

# CSVファイルにデータを書き込む関数
def save_to_csv(filename, arrival_times, total_distances_list):
    # カレントディレクトリを取得
    current_directory = os.getcwd()
    # 保存先の相対パス
    relative_path = "result(CSV)"
    # 保存フォルダの絶対パスを作成
    save_folder = os.path.join(current_directory, relative_path)
    os.makedirs(save_folder, exist_ok=True) #ディレクトリがない場合作成
    
    filename = f"result {road_filename} {num_cars} {num_obstacles} fake_car{num_fakecar}seed{a}.csv"

    with open(os.path.join(save_folder, filename), mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Car Index", "Arrival Time", "Total Distance"])  # ヘッダー行を追加（適宜変更）
        print("csvへの書き込み中")

        for i, (arrival_time, moving_distance) in enumerate(zip(arrival_times, total_distances_list)):
            writer.writerow([i + 1, arrival_time, moving_distance])  # 車両のインデックス、到着時刻、移動距離を別々の列に書き込む


ani = animation.FuncAnimation(fig, update, init_func=init, blit=True, interval=10, repeat=False, cache_frame_data=False)

plt.show()
