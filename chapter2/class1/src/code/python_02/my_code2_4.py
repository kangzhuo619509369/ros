#Number数据类型
'''
1.Int类型     整型
2.Float类型   浮点数
3.Bool类型    布尔型
4.Complex类型 复数
'''

'''
#Int类型   整型也就是整数  关键字 int 或者integer
#方式1：十进制声明   0~9
intvar = 250
#打印变量的值
print(intvar)
#打印变量的类型
print(type(intvar))

#方式2：二进制声明  0~1
intvar = 0b101011
#打印变量的值
print(intvar)
#打印变量的类型
print(type(intvar))

#方式3：八进制声明  0~7
intvar = 0o147
#打印变量的值
print(intvar)
#打印变量的类型
print(type(intvar))

#方式4：十六进制声明 0~9A-F
intvar = 0x12af
#打印变量的值
print(intvar)
#打印变量的类型
print(type(intvar))
'''

'''
#Float类型  浮点型 就是小数  float或者real 或者double

#方法1：用小数声明
floatvar = 3.141592653
#打印变量的值
print(floatvar)
#打印变量的类型
print(type(floatvar))

#方法2:科学计数声明
floatvar = 25041e-2 # 25041 乘以10的-2次方
#打印变量的值
print(floatvar)
#打印变量的类型
print(type(floatvar))
'''

'''
#Bool类型 布尔类型
#只有两个值：True 和 False
boolvar = True
#打印变量的值
print(boolvar)
#打印变量的类型
print(type(boolvar))
'''

#Complex类型  复数类型  complex
#复数表示所有的数字，由实数和虚数两部分组成
#实数：真实存在的数字就是实数
#虚数：不存在的数字就是虚数。（假设一个数字的平方是-1，这个数就是虚数的单位j）

#方法1：使用表达式
comvar = 55 + 2j
#打印变量的值
print(comvar)
#打印变量的类型
print(type(comvar))

#方法2：使用特定功能
comvar = complex(23,99)
#打印变量的值
print(comvar)
#打印变量的类型
print(type(comvar))