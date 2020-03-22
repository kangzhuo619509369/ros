'''
#print函数  :输出一段数据
print('**********')
print('**********')
print('**********')
print('**********')
#查看print函数的结构
help(print)

#print输出数据不换行
print('##########',end = '')
print('@@@@@@@@@@',end = '')
print('&&&&&&&&&&',end = '')
'''

#使用偏函数修改函数的默认值
#必须导入额外的模块
import functools #函数工具模块

#定义一个偏函数
print2 = functools.partial(print,end = '')
#注意 print2函数和print函数功能完全一致，end的默认值不一样
print('#########')
print('@@@@@@@@@')
print('&&&&&&&&&')


print2('#########')
print2('@@@@@@@@@')
print2('&&&&&&&&&')