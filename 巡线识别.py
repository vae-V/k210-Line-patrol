import sensor, image, time,math#, car, text , pid   #导入库

sensor.reset() # 初始化摄像头
sensor.set_framesize(sensor.QVGA)#设置图像的大小
sensor.set_pixformat(sensor.RGB565)#设置为彩色
sensor.skip_frames(time = 2000) #等待2秒感光器校准亮度/白平衡
sensor.set_auto_gain(False) #锁定自动亮度
sensor.set_auto_whitebal(False) #锁定自动白平衡
#sensor.set_windowing((0,180,320,60))  #剪裁画面
midpoint=160    #设置画面x轴向中值
sensor.set_auto_exposure(False,exposure_us=4000)    #设置固定帧速
sensor.set_contrast(3)  #设置对比度，范围-3~3
sensor.set_saturation(3)    #设置饱和度，范围-3~3


threshold=[(0,128),(0,128),(0,128),(0,128),(0,128),(0,128)]#创建阈值变量
line_cx=[0,0,0,0,0,0]    #记录六个线的x轴点
line_cy=[0,0,0,0,0,0]    #记录六个线的y轴点
line_area=[[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]#记录六个识别块的参数
angle=0 #
line_none=0 #记录没有发现黑线段的次数

clock = time.clock()    #声明时钟，用于获取帧速

def get_k(n,dataCol_X,dataRow_Y,none):  #线性回归求斜率
    k = 0         #斜率
    aveCol_X = 0  #列的平均值x
    aveRow_Y = 0  #行的平均值y
    sum_XY = 0    #行列的总和xy
    sumRow_Y = 0  #行的总和y
    sumCol_X = 0  #列的总和x
    sumCol_X2 = 0 #列的总和x^2
    for i in range(0,n):
       if dataRow_Y[i]>-1:
          sumCol_X += dataCol_X[i]  #求列x的总和
          sumRow_Y += dataRow_Y[i]  #求行y的总和
          sumCol_X2 += dataCol_X[i] * dataCol_X[i] #求x^2的总和
          sum_XY += (dataCol_X[i] * dataRow_Y[i])  #求xy的总和
    aveCol_X = float(sumCol_X /(n-none)) #求平均值
    aveRow_Y = float(sumRow_Y /(n-none))

    if(sumCol_X2 - aveCol_X * aveCol_X * (n-none)):

      k = (sum_XY - aveCol_X * aveRow_Y * (n-none)) / (sumCol_X2 - aveCol_X * aveCol_X * (n-none)) #根据公式求斜率
    else:
      k=0
    return k

def get_angle(ture,move):  #获取转向角度，+左-右
    ture_angle=45*ture
    move_angle=45*(move/160)
    angle=0.7*ture_angle+0.3*move_angle
    return angle



#主循环
while True:
    line_move=0 #左右偏移量
    line_ture=0
    line_px=0
    line_py=0
    clock.tick()    #用于统计帧速的程序，取消注释可用
    img = sensor.snapshot().binary([(0,40)],invert=1) #获取感光器画面
    for n in range(0,6):    #循环6次，分别对应画面被分割的6各区域
        threshold[n]=[0,50] #默认阈值为0~60。黑白画面，阈值只需要最小值和最大值即可。
        while True: #自动收束阈值的循环
            #按阈值寻找色块，roi是有效区，根据上一级for循环而定，
            #x_stride、y_stride、pixels_threshold分别是过滤掉的x轴、y轴、面积尺寸。
            blobs = img.find_blobs([threshold[n] ],roi=(0,n*40,320,40),\
                    x_stride=5,y_stride=5,pixels_threshold=50)#设置过滤长或宽小于5,像素小于50的色块
            if blobs:   #如果有结果
                if len(blobs)>1:    #多个结果
                    if threshold[n][1]>20:  #阈值大于20之前
                        threshold[n][1]-=5  #每次减5（收束阈值）
                    else:   #阈值等于20了还有多个结果
                        break   #跳出
                else:   #只有一个结果
                    break   #跳出
            else:   #没有结果
                break   #跳出
        if blobs:#如果有识别结果
            blob=max(blobs, key = lambda b: b.pixels())#在识别结果中，以面积(像素)为依据，找最大值。
            line_area[n]=blob.rect()    #记录当前色块所在区域
            line_cx[n]=blob.cx()    #记录色块的中心点
            line_cy[n]=blob.cy()    #记录色块的中心点
            #在相应位置画方块。
            img.draw_rectangle(int(line_area[n][0]),int(line_area[n][1]),int(line_area[n][2]),int(line_area[n][3]),color=(255,0,0))
            if n>0 and line_area[n]!=-1 and line_area[n-1]!=-1:#如果不是第一个点，且前后的点有数据
                #在识别到的黑线中心画线。
                img.draw_line(int(line_cx[n]), int(line_cy[n]) , int(line_cx[n-1]) , int(line_cy[n-1]),color=(255,0,0), thickness=2)
            #如果色块宽度在画面宽度的0.375到0.875之间（角度很大的弧线或直角）
            #注：下面数值是以画面一般的数值作为基数计算，所以相对于完整画面，数值在37.5%到87.5%之间。
            if midpoint*0.75<blob.w()<midpoint*1.75:
                line_cx[n]=line_cx[n]*2
        else:
            line_area[n]=-1 #没有识别到结果装填一个负值，在后文可以判断是否识别到了结果
            line_cx[n]=-1
            line_cy[n]=-1
            line_none+=1 #记录没有发现黑线段的次数

    if line_none<=2:
       for n in range(5,-1,-1):
           if line_cx[n]>-1:  #如果有数值
              line_move=(line_move+(line_cx[n])) #累加再取平均值
       line_move=round((-(line_move/(6-line_none)-160)),2)
       line_ture=round(get_k(6,line_cy,line_cx,line_none),2)
       #print(line_ture,line_move)
       angle=get_angle(line_ture,line_move)
       print(angle)

#print(clock.fps())  #打印帧速，方便在编程软件中调试.

