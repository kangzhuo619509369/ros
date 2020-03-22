'''
#关键字收集参数
def showinfo(**person):
    #打印person接受的结果
    print(person)
    print('参与大会的人员有' , person.keys())
    print('他们的体重分别是',person.values())

#调用函数
showinfo(ml = 90,xm = 120 ,xh = 98 ,xg = 145,gm = 90)#关键字参数
'''
#注意事项

#关键字收集参数
def showinfo(xh,**person):
    #打印person接受的结果
    print(person)
    print('参与大会的人员有' , person.keys())
    print('他们的体重分别是',person.values())

#调用函数
showinfo(ml = 90,xm = 120 ,xh = 98 ,xg = 145)#关键字参数

