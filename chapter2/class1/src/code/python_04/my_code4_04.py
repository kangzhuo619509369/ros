#定义函数
#定义一个人物角色，并且设置装备
def creatPerson(mz,xl,yf,wq,xz):
    print('头上戴着'+ mz )
    print('脖子上挂着' + xl)
    print('身上穿着' + yf)
    print('手上拿着' + wq)
    print('脚上穿着' + xz)

#调用函数（理想）
#creatPerson('三级头','钻石项链','锁子甲','金丝大环刀','草鞋')

#调用函数（出现顺序错误）
#creatPerson('钻石项链','锁子甲','金丝大环刀','草鞋','三级头')

#调用函数（使用关键字参数）
creatPerson(xl = '钻石项链', wq = '金丝大环刀', yf = '锁子甲',xz = '草鞋',mz = '三级头')