class block_object:
    def __init__(self, block, name, color, distance, plan_penalty = 0):
        self.name = name
        self.color = color
        self.distance = distance
        self.plan_penalty = plan_penalty
        self.block = block

    def __repr__(self):
        return "This block %s in %s has a distance %f" % (self.name, self.color,self.distance)

    def get_block(self):
        return self.block
    def get_name(self):
        return self.name
    def get_color(self):
        return self.color
    def get_distance(self):
        return self.distance
    def get_plan_panelty(self):
        return self.plan_panelty





