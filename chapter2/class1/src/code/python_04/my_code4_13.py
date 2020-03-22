
#整个页面当作->国家
leader = '主席'

#定义一个省
def shandong():
    print('此处是山东省')
    print(leader)
    #定义一个市
    def jinan():
        print('此处是济南市')
        print(leader)

        #定义一个区域
        def huaiyin():
            print('此处是槐荫区')
            print(leader)

         #调用槐荫区
        huaiyin()

    #调用济南函数（必须在山东的函数内）
    jinan()


#调用山东函数
shandong()



