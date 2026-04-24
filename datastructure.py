import random as rand

class Path_Node:
    def __init__(self, robot):
        self.r_id = robot
        self.index = []
        self.prev_node = []
        self.next_node = []
        
    def pop(self):
        self.index.pop(0)
        self.prev_node.pop(0)
        self.next_node.pop(0)


class Node:
    def __init__(self, key):
        self.key = key
        self.data = Path_Node(key)
        self.node_count = 1
        self.height = 0
        self.count = 1
        self.parent = None
        self.right = None
        self.left = None
        
    def __str__(self):
        pass
        return f"D:{self.data} Right:{self.right} Left:{self.left}"
    
    """
    Update height of node
    """
    def update_parameter(self):
        if (self.right == None) and (self.left == None):
            self.height = 0
            self.node_count = 1
        elif (self.right == None) and (self.left is not None):
            self.height = self.left.height + 1
            self.node_count = self.left.node_count + 1
        elif (self.right is not None) and (self.left == None):
            self.height =  self.right.height + 1
            self.node_count = self.right.node_count + 1
        elif (self.right is not None) and (self.left is not None):
            self.height = max(self.right.height, self.left.height) + 1
            self.node_count = self.right.node_count + self.left.node_count + 1
            
    """
    Insert key
    """
    def insert(self, key):
        #no repetition
        if key == self.key:
            self.count += 1
            return self
        #left side
        elif key < self.key:
            if self.left is not None:
                new_node = self.left.insert(key)
                self.update_parameter()
                return new_node
            else:
                self.left = Node(key)
                self.left.parent = self
                self.update_parameter()
                return self.left
        #right side
        elif key > self.key:
            if self.right is not None:
                new_node = self.right.insert(key)
                self.update_parameter()
                return new_node
            else:
                self.right = Node(key)
                self.right.parent = self
                self.update_parameter()
                return self.right
    
    """
    Return rightmost element
    """
    def rightmost(self):
        if self.right is not None:
            return self.right.rightmost()
        else:
            return self
        
    """
    Return leftmost element
    """
    def leftmost(self):
        if self.left is not None:
            return self.left.leftmost()
        else:
            return self
    
    """
    Return successor for this node
    """
    def successor(self):
        if self.right is not None:
            return self.right.leftmost()
        else:
            return self
    
    """
    Return predecessor for this node
    """
    def predecessor(self):
        if self.left is not None:
            return self.left.leftmost()
        else:
            return self
        
    def update_parent(self, node):
        if self.parent.key < self.key:
            self.parent.right = node
        elif self.parent.key > self.key:
            self.parent.left = node
            
    def update_all_parent(self):
        self.update_parameter()
        if self.parent is not None:
            self.parent.update_all_parent()
        else:
            return
        
    """
    Find key and return pointer
    """
    def find(self, key):
        if self.key == key:
            return self
        elif key < self.key:
            if self.left is not None:
                return self.left.find(key)
            else:
                #print("No key found")
                return None
        elif key > self.key:
            if self.right is not None:
                return self.right.find(key)
            else:
                #print("No key found")
                return None
            
    def delete(self, key):
        node = self.find(key)
        if node is not None:
            #print("No key found")
            return (False, None)
        else:
            #update count to zero
            node.count = 1
            return self.remove(key)
            
    """
    Remove key
    """
    def remove(self, key):
        if self.key == key:
            if self.count > 1:
                self.count -= 1
                self.data.pop()
                return (False,None)
            else:
                if self.parent is not None:
                    #no child 
                    if (self.left == None) and (self.right == None):
                        self.update_parent(None)
                        return (False,None)
                    #left child
                    elif (self.left is not None) and (self.right == None):
                        self.left.parent = self.parent
                        self.update_parent(self.left)
                        return (False,None)
                    #right child
                    elif (self.left == None) and (self.right is not None):
                        self.right.parent = self.parent
                        self.update_parent(self.right)
                        return (False,None)
                    #two child
                    else:
                        self.right.parent = self.parent
                        self.update_parent(self.right)     
                        pred = self.right.predecessor()
                        pred.left = self.left
                        self.left.parent = pred
                        pred.update_all_parent()
                        return (False,None)
                else:
                    #no child
                    if (self.left == None) and (self.right == None):
                        return (True,None)
                    #left child
                    elif (self.left is not None) and (self.right == None):
                        new_root = self.left
                        self.left.parent = self.parent
                        self.left = None
                        return (True,new_root)
                    #right child
                    elif (self.left == None) and (self.right is not None):
                        new_root = self.right
                        self.right.parent = self.parent
                        self.right = None
                        return (True,new_root)
                    #two child
                    else:
                        new_root = self.right
                        self.right.parent = self.parent 
                        pred = self.right.predecessor()
                        pred.left = self.left
                        self.left.parent = pred
                        pred.update_all_parent()   
                        self.right = None
                        self.left = None
                        return (True,new_root)
                    
        elif key < self.key:
            if self.left is not None:
                new_root = self.left.remove(key)
                self.update_parameter()
                return new_root
            else:
                #print("No key found")
                return (False,None)
        elif key > self.key:
            if self.right is not None:
                new_root = self.right.remove(key)
                self.update_parameter()
                return new_root
            else:
                #print("No key found")
                return (False,None)
    
    """
    Return list of node pointer
    """
    def traversal(self, iterator):
        if self.left is not None:
            self.left.traversal(iterator)
        iterator.append(self)
        if self.right is not None:
            self.right.traversal(iterator)
        
    def inorder(self):
        if self.left is not None:
            self.left.inorder()
        print(f"D:{self.key} C:{self.count} h:{self.height} N:{self.node_count}")
        if self.right is not None:
            self.right.inorder()
            
    def preorder(self):
        print(f"D:{self.key} C:{self.count} h:{self.height} N:{self.node_count}")
        if self.left is not None:
            self.left.preorder()
        if self.right is not None:
            self.right.preorder() 
            
    def postorder(self):
        if self.left is not None:
            self.left.postorder()
        if self.right is not None:
            self.right.postorder() 
        print(f"D:{self.key} C:{self.count} h:{self.height} N:{self.node_count}")


class BST:
    def __init__(self):
        self.root = None
        self.iterator = []

    def insert(self, data):
        if self.root is not None:
            node = self.root.insert(data)
            return node
        else:
            self.root = Node(data)
            return self.root
            
    def remove(self, data):
        if self.root is not None:
            (change, new_root) = self.root.remove(data)
            if change:
                self.root = new_root
        else:
            print("Empty tree")
            
    def delete(self, data):
        if self.root is not None:
            (change, new_root) = self.root.delete(data)
            if change:
                self.root = new_root
        else:
            print("Empty tree")
            
    def get_iterator(self):
        self.iterator = []
        if self.root is not None:
            self.root.traversal(self.iterator)
        return self.iterator
    
    def find(self, key):
        if self.root is not None:
            return self.root.find(key)
        else:
            return None
        
    def __len__(self):
        if self.root is not None:
            return self.root.node_count
        else:
            return 0

"""
Unit Testcases
"""
def BST_Test():
    #testcase for working of datastructure
    for _ in range(1000):
        #Unit test I: Insertion by checking preorder in sorted order
        a = rand.randint(0,100)
        test = [a]
        root = BST()
        root.insert(a)
        for i in range(100):
            key = rand.randint(0, 100)
            node = root.insert(key)
            test.append(key)
            node.data.index.append(rand.randint(0,1000))
            node.data.prev_node.append(rand.randint(0,100))
            node.data.next_node.append(rand.randint(0,100))
        
        iterator = root.get_iterator()
        data = []
        for node in iterator:
            data.append([node.key, node.count])
        #root.inorder()
        test.sort()

        k = 0
        for i in range(len(data)):
            count = data[i][1]
            for j in range(count):
                if data[i][0] != test[k]:
                    print(f"{k} Error D:{data[i][0]} T:{test[k]}")
                k += 1
            
        #Unit test II: Check total individual node count
        unique = set()
    
        for val in test:
            unique.add(val)
        
        if len(unique) != len(root):
            print(f"Error D:{len(root)} T:{len(unique)}")
        
        
        #Unit test III: Remove any random node and check node_count
        while len(test) > 0:
            idx = rand.randint(0, len(test)-1)
            node = test[idx]
            #print(f"Removing node {node}")
            test.remove(node)

            root.remove(node)
            
            unique = set()
    
            for val in test:
                unique.add(val)
            
            if len(unique) != len(root):
                print(f"Error D:{len(root)} T:{len(unique)}")