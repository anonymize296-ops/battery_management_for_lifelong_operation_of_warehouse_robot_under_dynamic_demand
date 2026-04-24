# import imp
import random as rand
import time 
from collections import deque

class Node:
    def __init__(self, data):
        self.data = data
        self.node_count = 1
        self.height = 0
        self.count = 1
        self.parent = None
        self.right = None
        self.left = None
        
    def __str__(self):
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
    def insert(self, data):
        #no repetition
        if data.key == self.data.key:
            self.count += 1
            return self
        #left side
        elif data.key < self.data.key:
            if self.left is not None:
                new_node = self.left.insert(data)
                self.update_parameter()
                return new_node
            else:
                self.left = Node(data)
                self.left.parent = self
                self.update_parameter()
                return self.left
        #right side
        elif data.key > self.data.key:
            if self.right is not None:
                new_node = self.right.insert(data)
                self.update_parameter()
                return new_node
            else:
                self.right = Node(data)
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
        if self.parent.data.key < self.data.key:
            self.parent.right = node
        elif self.parent.data.key > self.data.key:
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
    def find(self, data):
        if self.data.key == data.key:
            return self
        elif data.key < self.data.key:
            if self.left is not None:
                return self.left.find(data)
            else:
                #print("No key found")
                return None
        elif data.key > self.data.key:
            if self.right is not None:
                return self.right.find(data)
            else:
                #print("No key found")
                return None
            
    """
    [a, ] return iterators from root to a or node ~> a 
    """
    def near(self, data, iterator):
        #print(f"Node:{self.data.key}:")
        if self.data.key == data.key:
            iterator.append(self)
        elif data.key < self.data.key:
            if self.left is not None:
                iterator.append(self)
                self.left.near(data, iterator)
            else:
                iterator.append(self)
        elif data.key > self.data.key:
            if self.right is not None:
                iterator.append(self)
                self.right.near(data, iterator)
            else:
                iterator.append(self)
        
    def delete(self, data):
        node = self.find(data)
        if node is not None:
            #print("No key found")
            return (False, None)
        else:
            #update count to zero
            node.count = 1
            return self.remove(data)
            
    """
    Remove key
    """
    def remove(self, data):
        if self.data.key == data.key:
            if self.count > 1:
                self.count -= 1
                #self.data.pop()
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
                    
        elif data.key < self.data.key:
            if self.left is not None:
                new_root = self.left.remove(data)
                self.update_parameter()
                return new_root
            else:
                #print("No key found")
                return (False,None)
        elif data.key > self.data.key:
            if self.right is not None:
                new_root = self.right.remove(data)
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
        print(f"D:{self.data} C:{self.count} h:{self.height} N:{self.node_count}")
        if self.right is not None:
            self.right.inorder()
            
    def preorder(self):
        print(f"D:{self.data.key} C:{self.count} h:{self.height} N:{self.node_count}")
        if self.left is not None:
            self.left.preorder()
        if self.right is not None:
            self.right.preorder() 
            
    def postorder(self):
        if self.left is not None:
            self.left.postorder()
        if self.right is not None:
            self.right.postorder() 
        print(f"D:{self.data.key} C:{self.count} h:{self.height} N:{self.node_count}")


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
    
    def find(self, data):
        if self.root is not None:
            return self.root.find(data)
        else:
            return None
        
    def range_tree(self, a, b):
        if self.root is not None:
            iterator = deque()
            self.root.near(a,iterator)
            iterator1 = deque()
            self.root.near(b, iterator1)
            
            while True:
                if len(iterator) <= 1:
                    break
                if len(iterator1) <= 1:
                    break
                if iterator[1].data.key == iterator1[1].data.key:
                    iterator.popleft()
                    iterator1.popleft()
                else:
                    break
            
            right_side = []
            
            for iters in iterator1:
                if a.key <= iters.data.key <= b.key:
                    right_side.append(iters)
                    
            left_side = []

            for iters in iterator:
                if a.key <= iters.data.key <= b.key:
                    left_side.append(iters)

          
            return (left_side, right_side)
        else:
            return None
        
    def __len__(self):
        if self.root is not None:
            return self.root.node_count
        else:
            return 0

class Y_Data:
    def __init__(self, x, y):
        self.key = y
        self.x = []
        self.y = y
        
    def __str__(self):
        return f"X:{self.x},Y:{self.y}"

    def insert(self, x):
        self.x.append(x)

class XY_Data:
    def __init__(self, x):
        self.key = x
        self.x = x
        self.y = BST()
        self.child_y_sort = BST()
        
    def __str__(self):
        self.y.get_iterator()
        out = ""
        for iters in self.y.iterator:
            out += f"X:{self.x} Y:{iters.data.y},"
        return out
    
    def y_sort(self, node):
        iters = []
        node.traversal(iters)
        
        for it in iters:
            for y in it.data.y:
                self.child_y_sort.insert()
                
    def insert(self, y):
        self.y.insert(Y_Data(self.x, y))
        iters = self.y.find(Y_Data(self.x, y))
        iters.data.insert(self.x)

class Range_Tree:
    def __init__(self):
        self.points = None 
        self.x_sort = BST()
        self.inside_points = {}

    def set_points(self, points):
        self.points = points
        
    def sort_x(self):
        for pt in self.points:
            self.x_sort.insert(XY_Data(pt[0]))
            iters = self.x_sort.find(XY_Data(pt[0]))
            iters.data.insert(pt[1])
            
    def get_and_store_ysubtree(self):
        iters = []
        self.x_sort.root.traversal(iters)
        for it in iters:
            ite = []
            it.traversal(ite)
            for i in ite:
                i.data.y.get_iterator()
                for j in i.data.y.iterator:
                    it.data.child_y_sort.insert(Y_Data(i.data.x, j.data.y))
                    check = it.data.child_y_sort.find(Y_Data(i.data.x,j.data.y))
                    check.data.insert(i.data.x)
            
    def display(self):
        iters = []
        self.x_sort.root.traversal(iters)

        for it in iters:
            print("xxxxxxxxxxx")
            print(f"{it.data.x}:")
            ite = []
            it.data.child_y_sort.root.traversal(ite)
            for i in ite:
                print(f"X:{i.data.x}, Y:{i.data.y}")
            print("-----------")
    
    def get_all_right(self, left):
        count = 0
        for it in left:
            for x in it.data.x:
                self.inside_points[(x, it.data.y)] = 1
            if count > 0:
                if it.right is not None:
                    right_iter = []
                    it.right.traversal(right_iter)
                    for r_iter in right_iter:
                        for x in r_iter.data.x:
                            self.inside_points[(x,r_iter.data.y)] = 1
                                
            count += 1
            
    def get_all_left(self, right):
        again_count = 0
        for it in right:
            for x in it.data.x:
                self.inside_points[(x, it.data.y)] = 1
            if again_count > 0:
                if it.left is not None:
                    left_iter = []
                    it.left.traversal(left_iter)
                    for l_iter in left_iter:
                        for x in l_iter.data.x:
                            self.inside_points[(x, l_iter.data.y)] = 1
            again_count += 1
        
                
    def box_range(self, x_min, x_max, y_min, y_max):
        self.inside_points = {}
        
        (left_side, right_side) = self.x_sort.range_tree(XY_Data(x_min), XY_Data(x_max))
        count = 0
        for iters in right_side:
            (left, right) = iters.data.y.range_tree(Y_Data(iters.data.x, y_min), Y_Data(iters.data.x, y_max))
            self.get_all_left(right)
            self.get_all_right(left)
            
            if iters.left is not None:
                if count > 0:
                    (left, right) = iters.left.data.child_y_sort.range_tree(Y_Data(iters.data.x, y_min),Y_Data(iters.data.x, y_max))
                    self.get_all_left(right)
                    self.get_all_right(left)
                    
            count += 1
        
         
        count = 0
        for iters in left_side:
            (left, right) = iters.data.y.range_tree(Y_Data(iters.data.x, y_min), Y_Data(iters.data.x, y_max))
            self.get_all_left(right)
            self.get_all_right(left)
            
            if iters.right is not None:
                if count > 0:
                    (left, right) = iters.right.data.child_y_sort.range_tree(Y_Data(iters.data.x, y_min),Y_Data(iters.data.x, y_max))
                    self.get_all_left(right)
                    self.get_all_right(left)
                    
            count += 1      


"""
Unit testcase for range tree
""" 
def Range_Tree_Test():
    pree = 0.
    r_time = 0.
    b_time = 0.

    for j in range(30):
        #unit test
        points = []

        sample_length = rand.randint(100, 2000)
        for i in range(sample_length):
            x = rand.randint(150,800)
            y = rand.randint(10,80)
            points.append([x,y])
    
        start = time.time()
        a = Range_Tree(points)
        a.sort_x()
        a.get_and_store_ysubtree()
        end = time.time()
        pree += end - start 

        for i in range(1000):
            win_x = [rand.randint(150,800), rand.randint(150,800)]
            win_y = [rand.randint(10,80), rand.randint(10,80)]
        
            start = time.time()
            a.box_range(min(win_x), max(win_x), min(win_y), max(win_y))
            end = time.time()
        
            r_time += (end - start)
        
            start = time.time()
            output_pt = {}
            for pt in points:
                if (min(win_x) <= pt[0] <= max(win_x)) and (min(win_y) <= pt[1] <= max(win_y)) :
                    output_pt[(pt[0],pt[1])] = 1
            end = time.time()
        
            b_time += end - start

            if len(output_pt) != len(a.inside_points):
                print("Different length")
                break
        
            for pt in output_pt:
                if (pt[0],pt[1]) not in a.inside_points:
                    print("Different output pts")
                    break
    
    print(f"Time taken by range tree {r_time} and pre-processing {pree}")
    print(f"Time taken by search {b_time}")