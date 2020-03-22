'''
#列表类型  List     一组数据的有序组合
#            0      1        2        3     4
listvar = ['貂蝉','西施','杨玉环','王昭君',4]
#           -5      -4       -3         -2   -1
#打印变量的值
print(listvar)
#打印变量的类型
print(type(listvar))

#访问列表中某个数据
print(listvar[2])
print(listvar[-3])

#修改列表中的某个数据
listvar[2] = '杨贵妃'
print(listvar)
'''

'''
#元组类型  Tuple 一组数据的有序组合 : 元组不可以修改
#               0       1          2         3       4    5
tuplevar = ('西游记','红楼梦','三国演义','水浒传',3.15,True)
#              -6       -5         -4        -3      -2   -1
#打印变量的值
print(tuplevar)
#打印变量的类型
print(type(tuplevar))

#访问元组中的某个数据
print(tuplevar[1])
print(tuplevar[-5])

#修改：修改会报错！
#tuplevar[1] = '青楼梦'
'''

'''
#集合类型   Set  一组[特定数据]的无序组合
#集合中的数据不会重复
#集合没有顺序！

setvar = {'夏侯惇','大乔','曹操','大乔','关银屏','大乔','小乔'}
#打印变量的值
print(setvar)
#打印变量的类型
print(type(setvar))
'''

#字典类型  Dict   一组由键和值组成的数据

dictvar = {'行者':'武松','小旋风':'柴进','智多星':'吴用'}
#打印变量的值
print(dictvar)
#打印变量的类型
print(type(dictvar))

#访问字典中的某个数据
print(dictvar['智多星'])