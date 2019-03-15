class FoodItem(object):
    def __init__(self, name, description, id):
        self.name = name
        self.description = description
        self.id = id
    
    def __str__(self):
        return "FoodItem(name='%s', description='%s', id=%d)" % (self.name, self.description, self.id)