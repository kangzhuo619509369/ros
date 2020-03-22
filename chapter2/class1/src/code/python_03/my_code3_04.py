#for..in 循环
'''
#           0      1      2      3         4        5
fruit = ['苹果','榴莲','樱桃','水蜜桃','猕猴桃','西瓜']

#while循环操作
i = 0
while i <= 5:
    print(fruit[i])
    i += 1
'''
'''
1.容器长度不确定（后期可以操作）
2.有些容器没有编号
'''


#使用for..in循环解决问题
'''
#           0      1      2      3         4        5
fruit = ['苹果','榴莲','樱桃','水蜜桃','猕猴桃','西瓜']
#使用sg变量在fruit容器中获取数据
for sg in fruit:
    print(sg)
'''
#for。。in循环的终止条件是所有数据都被遍历

#for...in循环遍历字典
fruit = {'a':'苹果','b':'榴莲','c':'樱桃','d':'水蜜桃','e':'猕猴桃','f':'西瓜'}

#直接遍历字典 变量仅仅获取键（目录部分）
for i in fruit:
    print(i)

#专门遍历字典的值（数据）
for i in fruit.values():
    print(i)

#专门遍历字典的键（目录）
for i in fruit.keys():
    print(i)

#同时遍历字典的键和值
for k,v in fruit.items():
    print(k,v)