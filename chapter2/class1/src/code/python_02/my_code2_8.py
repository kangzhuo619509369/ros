#强制数据类型转换
'''
#1.int()    强制转换为整型  Int
#var = 88
#var = 8.9
#var = False
#var = 5 - 3j
#var = '99'#var = 'hero'
#var = [1,2,3,5,76]
#var = (1,2,3,54,6,7)
#var = {1,2,3,5,6}
#var = {'a':1,'b':2}
#打印原有数据类型和值
print(var,type(var))
#强制转换操作
newvar = int(var)
#打印转换之后的数据类型和值
print(newvar,type(newvar))
'''

'''
#2.float()  强制转换为浮点型 Float
#var = 88
#var = 8.9
#var = True
#var = 5 - 3j
#var = '9.9'#var = 'hero'
#var = [1,2,3,5,76]
#var = (1,2,3,54,6,7)
#var = {1,2,3,5,6}
#var = {'a':1,'b':2}

#打印原有类型和值
print(var,type(var))
#强制转换过程
newvar = float(var)
#打印转换之后的类型和值
print(newvar,type(newvar))
'''

'''
#3.bool()   强制转换为布尔类型 Bool
#var = 0
#var = 0.0
#var = False
#var = 0 + 0j
#var = ''
#var = []
#var = ()
#var = set()
#var = {'a':1,'b':2}

#打印原有类型和值
print(var,type(var))
#强制转换过程
newvar = bool(var)
#打印转换之后的类型和值
print(newvar,type(newvar))
'''

'''
#4.complex()    强制转换为复数   Complex
#var = 88
#var = 8.9
#var = True
#var = 5 - 3j
#var = '9.9'#var = 'hero'
#var = [1,2,3,5,76]
#var = (1,2,3,54,6,7)
#var = {1,2,3,5,6}
#var = {'a':1,'b':2}
#打印原有数据类型和值
print(var,type(var))
#强制转换操作
newvar = complex(var)
#打印转换之后的数据类型和值
print(newvar,type(newvar))
'''

'''
#5.str()    强制转换为字符串  Str

#var = 19
#var = 7.8
#var = False
#var = 9 + 2j
#var = '小飞机'
#var = [1,2,3,5,67]
#var = (1,2,3,5,7) #-> '(1,2,3,5,7)'
#var = {1,2,3,5,6}
#var = {'a':1,'b':2}
#打印原有数据的类型和值
print(var,type(var))
#强制类型转换
newvar = str(var)
#打印转换之后的类型和值
print(newvar,type(newvar))
'''

'''
#6.list()   强制转换为列表   List

#var = '唧唧复唧唧，木兰当户织'
#var = [2,3,5,6,7]
#var = (1,2,3,4,5,6,7,8,9)
#var = {1,2,3,5,6,8,9}
var = {'a':1,'b':2,'c':3}
#打印原有数据的类型和值
print(var,type(var))
#强制类型转换
newvar = list(var)
#打印转换之后的类型和值
print(newvar,type(newvar))
'''

'''
#7.tuple()  强制转换为元组  Tuple
#var = '唧唧复唧唧，木兰当户织'
#var = [2,3,5,6,7]
#var = (1,2,3,4,5,6,7,8,9)
#var = {1,2,3,5,6,8,9}
#var = {'a':1,'b':2,'c':3}
#打印原有数据的类型和值
print(var,type(var))
#强制类型转换
newvar = tuple(var)
#打印转换之后的类型和值
print(newvar,type(newvar))
'''

'''
#8.set()    强制转换为集合   Set

#var = '唧唧复唧唧，木兰当户织'
#var = [2,3,5,6,7,2,3,5,6,7]
#var = (1,2,3,4,5,6,7,8,9,8,8,8,8)
#var = {1,2,3,5,6,8,9}
var = {'a':1,'b':2,'c':3}
#打印原有数据的类型和值
print(var,type(var))
#强制类型转换
newvar = set(var)
#打印转换之后的类型和值
print(newvar,type(newvar))
'''

#9.dict()   强制转换为字典   Dict
#var = 9.9
#var = '唧唧复唧唧，木兰当户织'
"""
var = [
    ['A',1],
    ['B',2],
    ['C',3],
    ['D',4]
]#二级列表

var = (
    (1,1.1),
    (2,2.2),
    (3,3.3)
)#二级元组
"""
var = {
    (1,2),
    (3,4)
}#二级容器
#var = {'a':1,'b':2,'c':3}
#打印原有数据的类型和值
print(var,type(var))
#强制类型转换
newvar = dict(var)
#打印转换之后的类型和值
print(newvar,type(newvar))