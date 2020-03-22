#成员检测运算
#in 检测是否存在于容器中
#列表
#IT = ['PHP','JAVA','PYTHON','C++','C#','C']
#元组
#IT = ('PHP','JAVA','PYTHON','C++','C#','C')
#集合
#IT = {'PHP','JAVA','PYTHON','C++','C#','C'}
#字符串
#IT = 'PHPJAVAPYTHONC++C#C'
#字典（字典的成员运算仅仅进行键的检测）
IT = {'a':'PHP','b':'JAVA','c':'PYTHON','d':'C++','e':'C#','f':'C'}
result = 'a' in IT
print(result)

#not in 检测是否不再容器当中
IT = {'PHP','JAVA','PYTHON','C++','C#','C'}
result = 'lua' not in IT
print(result)