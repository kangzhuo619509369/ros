'''
#定义带有参数的函数
def eatfruit(xm,xh):
    print('小明喜欢吃水果' + xm)
    print('小红喜欢吃水果' + xh)

#调用函数
eatfruit('西瓜','榴莲')
#注意：
    #1.实参传值给形参的时候按照位置编号传输的
    #2.实参必须根据形参的个数进行传值（普通参数）
'''

#参数的默认值
#定义一个函数
def buyPC(cpu = 'I5',memory = '4G',videocard = '970'):
    print('CPU是'+ cpu)
    print('内存是'+ memory)
    print('显卡是' + videocard)

#调用函数  传入实参 -> 使用的是实参值
#buyPC('I7-7700K','16G','1080ti')

#调用函数  不传入实参 -> 使用的是形参的默认值
#buyPC()

#调用函数  传入一个实参
#buyPC('I5','16G')


