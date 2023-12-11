import numpy as np

connectivity_threshold = 2000
GEO_LEO_connectivity_threshold = 38000
R_LEO_max = 400
R_GEO_max = 40
R_interSate = 1000
R_GEO_LEO_max = 100

class Link:
    def __init__(self, node1, node2, type_link):
        self.node1 = node1
        self.node2 = node2
        self.type_link = type_link
        self.available_keys = 0

        if type_link == 'LEO-GS':
          Ds = self.cal_poss_D()
          self.D_max = np.max(Ds)
          self.D_min = np.min(Ds)

          self.link_distance = self.cal_distance(0)
          self.secret_key_rate = R_LEO_max*(self.link_distance - self.D_max)/(self.D_min - self.D_max)

        if type_link == 'LEO-LEO':
          self.link_distance = self.cal_distance(0)
          self.secret_key_rate = R_interSate

        if type_link == 'GEO-GS':
          self.link_distance = self.cal_distance(0)
          self.secret_key_rate = 40

        if type_link == 'LEO-GEO':
          Ds = self.cal_poss_D()
          self.D_max = np.max(Ds)
          self.D_min = np.min(Ds)

          self.link_distance = self.cal_distance(0)
          self.secret_key_rate = R_GEO_LEO_max*(self.link_distance - self.D_max)/(self.D_min - self.D_max)

    def cal_distance(self, t):
        x1,y1,z1 = self.node1.dynamics[t]
        x2,y2,z2 = self.node2.dynamics[t]
        return np.sqrt((x1-x2)**2+(y1-y2)**2+(z1-z2)**2)

    def __str__(self):
      if self.node1.get_name() == 'GS2':
        return "Path : " + self.node2.get_name() + " -> " + self.node1.get_name() + "\nResources Remaining: " + str(self.available_keys) + "\nBandwidth: " + str(self.
                                                                                                                                                       secret_key_rate)
      else:
        return "Path : " + self.node1.get_name() + " -> " + self.node2.get_name() + "\nResources Remaining: " + str(self.available_keys) + "\nBandwidth: " + str(self.
                                                                                                                                                       secret_key_rate)
    def get_sourceName(self):
      return self.node1.get_name()

    def get_destName(self):
      return self.node2.get_name()

    def cal_poss_D(self):
      Ds = []
      for t in range(100):
        Ds.append(self.cal_distance(t))
      return Ds

    def check_connectivity(self):
      if self.type_link == 'LEO-LEO':
        return True
      elif self.type_link == 'GEO-GS':
        return True
      elif self.type_link == 'LEO-GS':
        return self.link_distance < connectivity_threshold
      elif self.type_link == 'LEO-GEO':
        return self.link_distance < GEO_LEO_connectivity_threshold

    def update(self, t):
      if self.type_link == 'LEO-GS':
        self.link_distance = self.cal_distance(t)
        self.secret_key_rate = R_LEO_max*(self.link_distance - self.D_max)/(self.D_min - self.D_max)
        self.available_keys = self.available_keys + self.secret_key_rate
        if self.available_keys > 5000:
          self.available_keys = 5000
      elif self.type_link == 'LEO-GEO':
        self.link_distance = self.cal_distance(t)
        self.secret_key_rate = R_GEO_LEO_max*(self.link_distance - self.D_max)/(self.D_min - self.D_max)
        self.available_keys = self.available_keys + self.secret_key_rate
        if self.available_keys > 1000:
          self.available_keys = 1000
      elif self.type_link == 'LEO-LEO':
        self.available_keys = self.available_keys + self.secret_key_rate
        if self.available_keys > 25000:
          self.available_keys = 25000
      elif self.type_link == 'GEO-GS':
        self.available_keys = self.available_keys + self.secret_key_rate
        if self.available_keys > 2000:
          self.available_keys = 2000


    def consume_key(self, N):
      self.available_keys -= N

class Node:
    def __init__(self, name, dynamics, type_node):
      self.name = name
      self.type_node = type_node
      self.dynamics = dynamics
      self.links = []

    def __str__(self):
      return "Node: "+self.name+", type: "+self.type_node

    def connect(self, node2):
      link = None
      if self.type_node == 'GS' and node2.type_node == 'LEO':
        link = Link(self, node2, 'LEO-GS')
      elif self.type_node == 'GS' and node2.type_node == 'GEO':
        link = Link(self, node2, 'GEO-GS')
      elif self.type_node == 'LEO' and node2.type_node == 'LEO':
        link = Link(self, node2, 'LEO-LEO')
      elif self.type_node == 'LEO' and node2.type_node == 'GEO':
        link = Link(self, node2, 'LEO-GEO')
      self.links.append(link)
      node2.links.append(link)
      return link

    def query_rightLink(self, node_name):
      for link in self.links:
        if link.get_destName() == node_name or link.get_sourceName() == node_name:
          return link

    def get_list_links(self):
      return self.links

    def get_name(self):
      return self.name

    def update(self, t):
      for link in self.links:
        link.update(t)