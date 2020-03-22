#break语句  破坏的意思
#break语句主要应用于循环结构当中，作用终止循环。

'''
#输出1-100的数字,遇到了44 就停止循环
no = 1
while no <= 100:
    #判断no是否是44
    if no == 44:
        break
    print(no)
    no += 1
'''

#continue 语句   继续
#循环中使用continue语句，作用是跳过本次循环，开始下一次循环

'''
#输出 1-100的数值,不要带有4的数值
no = 1
while no <= 100:
    #判断数值中是否有4
    if no % 10 == 4 or 40 <= no <=49:#都符合带有4的条件   【'4' in str(no)】
        #变量自增
        no += 1
        continue#执行了continue语句 立刻返回循环的开始位置
    print(no)
    no += 1
'''

#pass 语句  占位符号  保证语法不出错
if True:
    pass
else:
    print('*************')