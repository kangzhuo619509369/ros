#1.逻辑与运算（二目运算）-》有假则假
var1 = False
var2 = False
result = var1 and var2
print(result)
'''
True and True = True
False and True = False
True and False = False
False and False = False
'''


#2.逻辑或运算 -》 有真则真
var1 = False
var2 = False
result = var1 or var2
print(result)
'''
True or True = True
False or True = True
True or False = True
False or False = False
'''

#3.逻辑非运算（单目运算）-》真变假假变真
var = False
result = not var
print(result)

#4.逻辑异或运算-》相同为假不同为真
'''
True xor True = False
False xor False = False
True xor False = True
False xor True = True
'''
