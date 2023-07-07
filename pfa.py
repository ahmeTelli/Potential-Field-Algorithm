import numpy as np
import math
import vector3d
import numpy as np 
from scipy.spatial import distance

class PFA:
    def __init__(self,goal,obstacles) :
        self.k_rep = 10
        self.potential_cell = []
        self.goal = goal
        self.newPosition = self.goal
        
        self.obstacles = obstacles
        
        
        self.threshold = 0.1
        self.obstacle_distance = 1
        
    #iki nokta arası euclid mesafesi   
    def distance(self,agent,target):
        return distance.euclidean(agent, target)

    # Ajanın belirlenen mesafe kadar etrafında bulunan hücreleri belirler
    def allCell(self,agent):
        
        #belirlenen hücrelerin ajana uzaklığı
        per_of_cell = 0.2
        
        proble_cell_newPos = []
        
        agent_init_pos_x = agent[0]
        agent_init_pos_y = agent[1]
        agent_init_pos_z = agent[2]
        
        proble_cell_newPos.append([agent_init_pos_x+per_of_cell,agent_init_pos_y,agent_init_pos_z])
        proble_cell_newPos.append([agent_init_pos_x+per_of_cell,agent_init_pos_y+per_of_cell,agent_init_pos_z])
        proble_cell_newPos.append([agent_init_pos_x,agent_init_pos_y+1,agent_init_pos_z])
        proble_cell_newPos.append([agent_init_pos_x-per_of_cell,agent_init_pos_y+per_of_cell,agent_init_pos_z])
        proble_cell_newPos.append([agent_init_pos_x-per_of_cell,agent_init_pos_y,agent_init_pos_z])
        proble_cell_newPos.append([agent_init_pos_x-per_of_cell,agent_init_pos_y-per_of_cell,agent_init_pos_z])
        proble_cell_newPos.append([agent_init_pos_x,agent_init_pos_y-0.5,agent_init_pos_z])
        proble_cell_newPos.append([agent_init_pos_x+per_of_cell,agent_init_pos_y-per_of_cell,agent_init_pos_z])
    
        return proble_cell_newPos
    
    # Her hücre için potensiyel değeri hesaplar
    def computeCell(self,agent):
        
        # her hücre için ajanın olası potensiyel değeri
        prob_newAgentPosition = []
        
        cellKord = self.allCell(agent)
        
        newPosition = cellKord[0]
        
        for pos in cellKord:
            kord = self.sumPFA(pos)
            prob_newAgentPosition.append(kord)
        
        # karşılaştırma için belirlenen swap değeri    
        swap = prob_newAgentPosition[0]
        
        for index,cell in enumerate(prob_newAgentPosition):
            if index != 7 and swap >= cell:
                swap = cell
                newPosition = cellKord[index]

        return newPosition
    
    # Hefef noktanın uyguladığı çekme kuvveti
    # Bu kuvvet hedefe yakınlaştıkça azalmaktadır
    def cekiciKuvvet(self,target,agent):
        att_dist = distance.euclidean(agent, target)
        return (1/2)* 100 * math.sqrt(att_dist)
    
    
    # Engele uygulanan itme kuvveti
    def iticiKuvvet(self,uzaklik):
        if uzaklik <= self.obstacle_distance and uzaklik != 0:
            deger =(1/2)*((1/uzaklik-1/self.threshold)**2)
            return deger
        return 0
    
    def tümiticiKuvvetStatik(self,agent_pos):
        tümKuvvet = 0
        for obstacle in self.obstacles:
            print("asdasdasdasdasdasdasdasd")
            obs_uzaklik = self.distance(agent_pos,obstacle)
            deger = self.iticiKuvvet(obs_uzaklik)
            tümKuvvet += deger
        return tümKuvvet
        
    
    #Toplam potensiyel alan 
    def sumPFA(self,agent):
        deger = self.cekiciKuvvet(self.goal,agent) + self.tümiticiKuvvetStatik(agent) 
        print("degeri: {}".format(deger))
        return deger
    
    
    def agentDistance(self,agent,otherAgent):
        dist = self.distance(agent,otherAgent)
        if dist <= 1.0:
            return self.iticiKuvvet(dist)
        return 0

    # Hedefe doğru çizilmiş yol planı
    # Noktalar dizisi döndürür
    def pathPlan(self,pos):
        
        path = []
        
        while True:  
            newPos = self.computeCell(pos)
            mesafe = self.distance(pos,self.goal)

            path.append(newPos)
            if mesafe <= 0.2:
                break
            pos = newPos
        
        return path


#****************##
    #statik engellerin ajan ile arasında bulunan mesafe listesi
    def obstacleList(self,obstacle_s,pos):
        obstacles = []
        for obs in obstacle_s:
            obs_mesafe = self.distance(pos,obs)
            obstacles.append(obs_mesafe)
    
        return obstacles
    
    def checkObstacle(self,agent,obstacles):
        for obstacle in obstacles:
            obs_mesafe = self.distance(agent,obstacle)
            if obs_mesafe <= 0.50:
                return 50
        return 0
    
    
    def virtualObstacle():
        pass