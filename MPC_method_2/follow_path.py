import numpy as np
import matplotlib.pyplot as plt


def line_p(s_px,s_py,e_px,e_py,t):
  m1 = (e_py - s_py)/(e_px - s_px);
  if ((e_px - s_px) < 0) and ((e_py - s_py) > 0):
    teta = np.pi + np.arctan(m1);
  elif ((e_px - s_px) < 0) and ((e_py - s_py) < 0):
    teta = np.arctan(m1) -np.pi;
  else:
    teta = np.arctan(m1);
  dx = t*np.cos(teta);
  dy = t*np.sin(teta);
  D = np.sqrt((e_px - s_px)**2 + (e_py - s_py)**2);
  n = D//t;
  n = int(n)
  p_x = np.zeros((1,n));
  p_y = np.zeros((1,n));
  p_x[0,0] = s_px;
  p_y[0,0] = s_py;
  for i in range(n):
    s_px = s_px + dx;
    p_x[0,i] = s_px;
    s_py = s_py + dy;
    p_y[0,i] = s_py;
  return p_x, p_y

# -----------------------------------------------------------------------
teta_0_1 = np.full( 150, 0)
p1_y = 30*np.ones((1,150))
p1_x = np.array([np.arange(0,450,3)])
i_01y = 60*np.ones((1,150)) + 5*np.random.rand(150)
i_01x = np.array([np.arange(0,450,3)])
o_01y = 0*np.ones((1,150)) + 5*np.random.rand(150)
o_01x = np.array([np.arange(0,450,3)])


# -----------------------------------------------------------------------
teta_1 = (0.5*np.pi)/(0.5*np.pi*90/3)*np.arange(0,48,1)
p2_x = np.array([450*np.ones(48) + 90*np.sin(teta_1)])
p2_y = np.array([30*np.ones(48) + (90 - 90*np.cos(teta_1))])

i_02_1x , i_02_1y = line_p(450,60,492,77.5,3)
i_02_2x , i_02_2y = line_p(492,77.5,510,120,3)
i_02x = np.concatenate((i_02_1x[0], i_02_2x[0]))
i_02y = np.concatenate((i_02_1y[0], i_02_2y[0])) + 5*np.random.rand(len(i_02x))
o_o2_1x , o_o2_1y = line_p(450,0,520,36,3)
o_o2_2x , o_o2_2y = line_p(520,36,570,120,3)
o_02x = np.concatenate((o_o2_1x[0], o_o2_2x[0]))
o_02y = np.concatenate((o_o2_1y[0], o_o2_2y[0])) + 5*np.random.rand(len(o_02x))

# -----------------------------------------------------------------------
teta_1_2 = np.full( 60, (0.5*np.pi))
p3_x = 540*np.ones((1,60))
p3_y = np.array([np.arange(120 +(1 - 0.1238898038469)*3,60*3 + 120 + (1 - 0.1238898038469)*3,3)])

i_03x = 510*np.ones((1,60)) + 5*np.random.rand(60)
i_03y = np.array([np.arange(121,300,3)])
o_03x = 570*np.ones((1,60)) + 5*np.random.rand(60)
o_03y = np.array([np.arange(121,300,3)])

# -----------------------------------------------------------------------
teta_2 = np.arange((1 - 0.1238898038469 )*(3/90),(1 - 0.1238898038469 + 47)*(3/90),(3/90))
p4_x = np.array([540*np.ones(47) - (90 - 90*np.cos(teta_2))])
p4_y = np.array([300*np.ones(47) + (90*np.sin(teta_2))])
teta_2 = teta_2 + 1.570796*np.ones(47)

i_04_1x , i_04_1y = line_p(510,300,492,342.5,3)
i_04_2x , i_04_2y = line_p(492,342.5,450,360,3)
i_04x = np.concatenate((i_04_1x[0], i_04_2x[0]))
i_04y = np.concatenate((i_04_1y[0], i_04_2y[0])) + 5*np.random.rand(len(i_04x))
o_o4_1x , o_o4_1y = line_p(570,300,534,384,3)
o_o4_2x , o_o4_2y = line_p(534,384,450,420,3)
o_04x = np.concatenate((o_o4_1x[0], o_o4_2x[0]))
o_04y = np.concatenate((o_o4_1y[0], o_o4_2y[0])) + 5*np.random.rand(len(o_04x))

# -----------------------------------------------------------------------
teta_2_3 = np.full( 20, (np.pi))
p5_y = 390*np.ones((1,20))
p5_x = np.array([np.arange(447,387,-3)])

i_05x = np.array([np.arange(447,387,-3)])
i_05y = 360*np.ones((1,20)) + 5*np.random.rand(20)
o_05x = np.array([np.arange(447,387,-3)])
o_05y = 420*np.ones((1,20)) + 5*np.random.rand(20)

# -----------------------------------------------------------------------
teta_3 = 0.05*np.arange(0,32,1)
p6_y = np.array([390 - (60 - 60*np.cos(teta_3))])
p6_x = np.array([390*np.ones(32) - 60*np.sin(teta_3)])
p6 = np.concatenate((p6_x,p6_y),axis=0)
teta_3 = teta_3 + 3.141592*np.ones(32)

i_06_1x , i_06_1y = line_p(390,360,358,351,3)
i_06_2x , i_06_2y = line_p(358,351,350,300,3)
i_06x = np.concatenate((i_06_1x[0], i_06_2x[0]))
i_06y = np.concatenate((i_06_1y[0], i_06_2y[0])) + 5*np.random.rand(len(i_06x))
o_06_1x , o_06_1y = line_p(380,420,316,393,3)
o_06_2x , o_06_2y = line_p(316,393,300,300,3)
o_06x = np.concatenate((o_06_1x[0], o_06_2x[0]))
o_06y = np.concatenate((o_06_1y[0], o_06_2y[0])) + 5*np.random.rand(len(o_06x))

# -----------------------------------------------------------------------
y_d = 3 - 60*((np.pi/2) - 0.05*31)

teta_3_4 = np.full( 50, -0.5*np.pi)
p7_x = 330*np.ones((1,50))
p7_y = np.array([np.arange(330 - y_d,180,-3)])

i_07x = 350*np.ones((1,40)) + 5*np.random.rand(40)
i_07y = np.array([np.arange(300,180,-3)])
o_07x = 300*np.ones((1,40)) + 5*np.random.rand(40)
o_07y = np.array([np.arange(300,180,-3)])

# -----------------------------------------------------------------------
tt = 0.05*((3-y_d)/3)
teta_4 = np.arange(tt,np.pi/2+0.05,0.05)
p8_x = np.array([330 - (60 - 60*np.cos(teta_4))])
p8_y = np.array([180 - (60*np.sin(teta_4))])
teta_4 = -teta_4 - 1.570796*np.ones(32)

o_08_1x , o_08_1y = line_p(300,178,281,158,3)
o_08_2x , o_08_2y = line_p(281,158,260,150,3)
o_08x = np.concatenate((o_08_1x[0], o_08_2x[0]))
o_08y = np.concatenate((o_08_1y[0], o_08_2y[0])) + 5*np.random.rand(len(o_08x))
i_08_1x , i_08_1y = line_p(350,180,323,116,3)
i_08_2x , i_08_2y = line_p(323,116,290,90,3)
i_08x = np.concatenate((i_08_1x[0], i_08_2x[0]))
i_08y = np.concatenate((i_08_1y[0], i_08_2y[0])) + 5*np.random.rand(len(i_08x))

# -----------------------------------------------------------------------
teta_4_5 = np.full( 70, np.pi)
p9_y = 120*np.ones((1,70))
p9_x = np.array([np.arange(267,57,-3)])

i_09x = np.array([np.arange(287,57,-3)])
i_09y = 90*np.ones((1,77)) + 5*np.random.rand(77)
o_09x = np.array([np.arange(287,57,-3)])
o_09y = 150*np.ones((1,77)) + 5*np.random.rand(77)


# teta_5 = np.arange(0.1,np.pi,0.1)
# p10_x = np.array([60 - (30*np.sin(teta_5))])
# p10_y = np.array([120 + (30 - 30*np.cos(teta_5))])

# i_o10_1x, i_o10_1y = line_p(60,90,0,150,3)
# i_o10_2x, i_o10_2y = line_p(0,150,60,210,3)


# teta_5_6 = np.full( 20, 0)
# x_d = 3 - (np.pi - 3.1)*30
# p11_x = np.array([np.arange(60 + x_d,120,3)])
# p11_y = 180*np.ones((1,20))

# o_o11x,o_o11y = line_p(60,150,180,150,3)
# i_o11x, i_o11y = line_p(60,210,210,210,3)


# tt_0 = (3 - (120 - p11_x[0][19]))/(60);
# teta_6 = np.arange(tt_0,np.pi/2,0.05)
# p12_x = np.array([180 + (60*np.sin(teta_6))])
# p12_y = np.array([180 + (60 - 60*np.cos(teta_6))])

# o_o11_1x , o_o11_1y = line_p(181,150,244,176,3)
# o_o11_2x , o_o11_2y = line_p(244,176,270,210,3)


# teta_6_7 = np.full( 30, 0.5*np.pi)
# p13_y = np.array([np.arange(240,330,3)])
# p13_x = 240*np.ones((1,30))

# i_o13x = 210*np.ones((1,42))
# i_o13y = np.array([np.arange(210,330,3)])
# o_o13x = 270*np.ones((1,40))
# o_o13y = np.array([np.arange(210,328,3)])


# teta_7 = np.arange(0,np.pi/2,0.05)
# p14_x = np.array([240 - (60 - 60*np.cos(teta_7))])
# p14_y = np.array([330 + (60*np.sin(teta_7))])

# i_o14_1x , i_o14_1y = line_p(210,331,201,351,3)
# i_o14_2x , i_o14_2y = line_p(201,351,180,360,3)
# o_o14_1x , o_o14_1y = line_p(270,330,243,393,3)
# o_o14_2x , o_o14_2y = line_p(243,393,180,420,3)


# teta_7_8 = np.full( 30, np.pi)
# x_d2 = 3 - (np.pi/2 - teta_7[31])*60
# p15_x = np.array([np.arange(180 - x_d2,90,-3)])
# p15_y = 390*np.ones((1,30))

# i_o15y = 360*np.ones((1,30))
# i_o15x = np.array([np.arange(180,90,-3)])
# o_o15x = np.array([np.arange(180,90,-3)])
# o_o15y = 420*np.ones((1,30))


# tt_1 = (3 - (90 - p15_x[0][29]))/(30)
# teta_8 = np.arange(tt_1,np.pi/2,0.1)
# p16_x = np.array([90 - (30*np.sin(teta_8))])
# p16_y = np.array([390 - (30 - 30*np.cos(teta_8))])

# o_o16x , o_o16y = line_p(90,420,30,360,3)


# teta_8_9 = np.full( 20, -0.5*np.pi)
# p17_y = np.array([np.arange(360,300,-3)])
# p17_x = 60*np.ones((1,20))

# i_o17y = np.array([np.arange(359,301,-3)])
# i_o17x = 90*np.ones((1,20))
# o_o17y = np.array([np.arange(300,270,-3)])
# o_o17x = 30*np.ones((1,10))


# teta_9 = np.arange(0,np.pi/2,0.05)
# p18_x = np.array([60 - (60 - 60*np.cos(teta_9))])
# p18_y = np.array([300 - (60*np.sin(teta_9))])

# i_o18_1x , i_o18_1y = line_p(90,300,63,236,3)
# i_o18_2x , i_o18_2y = line_p(63,236,0,210,3)


# teta_9_10 = np.full( 20, np.pi)
# x_d3 = -(3 - p18_x[0][31])
# p19_x = np.array([np.arange(x_d3,-60,-3)])
# p19_y = 240*np.ones((1,20))

# i_o19y = 210*np.ones((1,20))
# i_o19x = np.array([np.arange(-1,-59,-3)])
# o_o19x, o_o19y = line_p(30,269,-60,255,3)


# tt_2 = (60 + p19_x[0][19])/(30)
# teta_10 = np.arange(tt_2,np.pi/2,0.1)
# p20_x = np.array([-60 - (30*np.sin(teta_10))])
# p20_y = np.array([240 - (30 - 30*np.cos(teta_10))])

# o_o20x , o_o20y = line_p(-60,255,-120,210,3)


# teta_10_11 = np.full( 30, -0.5*np.pi)
# p21_y = np.array([np.arange(207,120,-3)])
# p21_x = -90*np.ones((1,30))

# i_21y = np.array([np.arange(210,121,-3)])
# i_21x = -60*np.ones((1,30))
# o_o21x = -120*np.ones((1,30))
# o_o21y = np.array([np.arange(210,121,-3)])


# teta_11 = np.arange(0,np.pi/2,1/30)
# p22_x = np.array([-90 + (90 - 90*np.cos(teta_11))])
# p22_y = np.array([120 - (90*np.sin(teta_11))])

# i_o22_1x,i_o22_1y = line_p(-60,120,-42,77,3)
# i_o22_2x,i_o22_2y = line_p(-42,77,0,60,3)
# o_o22_1x,o_o22_1y = line_p(-120,120,-70.7,50,3)
# o_o22_2x,o_o22_2y = line_p(-70.7,50,0,0,3)

# teta_1 = teta_1
# teta_2 = teta_2 + 0.5*np.pi
# teta_3 = teta_3 - np.pi
# teta_4 = -teta_4 - 0.5*np.pi
# tets_5 = 0  # 
# teta_6 = teta_6
# teta_7 = teta_7 + 0.5*np.pi
# teta_8 = teta_8 - np.pi
# teta_9 = -teta_9 - 0.5*np.pi
# teta_10 = teta_10 - np.pi
# teta_11 = teta_11 - 0.5*np.pi

def path_coordinate() :
  # p_x = np.concatenate((p1_x,p2_x,p3_x,p4_x,p5_x,p6_x,p7_x,p8_x,p9_x,p10_x,p11_x,p12_x,p13_x,p14_x,p15_x,p16_x,p17_x,p18_x,p19_x,p20_x,p21_x,p22_x),axis =1)
  final_path_x = np.concatenate((p1_x,p2_x,p3_x,p4_x,p5_x,p6_x,p7_x,p8_x,p9_x),axis =1)
  final_path_x = final_path_x * 0.02

  # p_y = np.concatenate((p1_y,p2_y,p3_y,p4_y,p5_y,p6_y,p7_y,p8_y,p9_y,p10_y,p11_y,p12_y,p13_y,p14_y,p15_y,p16_y,p17_y,p18_y,p19_y,p20_y,p21_y,p22_y),axis =1)
  final_path_y = np.concatenate((p1_y,p2_y,p3_y,p4_y,p5_y,p6_y,p7_y,p8_y,p9_y),axis =1)
  final_path_y = final_path_y * 0.02

  # teta = np.concatenate((teta_0_1,teta_1,teta_1_2,teta_2,teta_2_3,teta_3,teta_3_4,teta_4,teta_4_5,teta_5,teta_5_6,teta_6,teta_6_7,teta_7,teta_7_8,teta_8,teta_8_9,teta_9,teta_9_10,teta_10,teta_10_11,teta_11))
  teta = np.concatenate((teta_0_1,teta_1,teta_1_2,teta_2,teta_2_3,teta_3,teta_3_4,teta_4,teta_4_5))
  teta = np.array([teta])
  # print(teta)
  # print(len(teta[0]), len(p_x[0]))

  path = np.concatenate((final_path_x, final_path_y, teta),axis = 0)

  final_iWall_x = np.concatenate( (i_01x[0], i_02x, i_03x[0], i_04x, i_05x[0], i_06x, i_07x[0], i_08x, i_09x[0]))
  final_iWall_y = np.concatenate( (i_01y[0], i_02y, i_03y[0], i_04y, i_05y[0], i_06y, i_07y[0], i_08y, i_09y[0]))
  final_oWall_x = np.concatenate( (o_01x[0], o_02x, o_03x[0], o_04x, o_05x[0], o_06x, o_07x[0], o_08x, o_09x[0]))
  final_oWall_y = np.concatenate( (o_01y[0], o_02y, o_03y[0], o_04y, o_05y[0], o_06y, o_07y[0], o_08y, o_09y[0]))

  return path



def mathplot() :
  final_path_x = np.concatenate( (p1_x[0], p2_x[0], p3_x[0], p4_x[0], p5_x[0], p6_x[0], p7_x[0], p8_x[0], p9_x[0]))
  final_path_y = np.concatenate( (p1_y[0], p2_y[0], p3_y[0], p4_y[0], p5_y[0], p6_y[0], p7_y[0], p8_y[0], p9_y[0]))

  final_iWall_x = np.concatenate( (i_01x[0], i_02x, i_03x[0], i_04x, i_05x[0], i_06x, i_07x[0], i_08x, i_09x[0]))
  final_iWall_y = np.concatenate( (i_01y[0], i_02y, i_03y[0], i_04y, i_05y[0], i_06y, i_07y[0], i_08y, i_09y[0]))
  final_oWall_x = np.concatenate( (o_01x[0], o_02x, o_03x[0], o_04x, o_05x[0], o_06x, o_07x[0], o_08x, o_09x[0]))
  final_oWall_y = np.concatenate( (o_01y[0], o_02y, o_03y[0], o_04y, o_05y[0], o_06y, o_07y[0], o_08y, o_09y[0]))

  plt.plot( final_path_x, final_path_y, label="Path")
  plt.plot( final_iWall_x, final_iWall_y, label="inner_wall")
  plt.plot( final_oWall_x, final_oWall_y, label="outer_wall")

  plt.show()



def map_data():
  final_path_x = np.concatenate( (p1_x[0], p2_x[0], p3_x[0], p4_x[0], p5_x[0], p6_x[0], p7_x[0], p8_x[0], p9_x[0]))
  final_path_y = np.concatenate( (p1_y[0], p2_y[0], p3_y[0], p4_y[0], p5_y[0], p6_y[0], p7_y[0], p8_y[0], p9_y[0]))
  teta = np.concatenate((teta_0_1,teta_1,teta_1_2,teta_2,teta_2_3,teta_3,teta_3_4,teta_4,teta_4_5))

  pathCoord = np.column_stack((final_path_x*0.02, final_path_y*0.02, teta))
  pathCoord = pathCoord.T

  final_iWall_x = np.concatenate( (i_01x[0], i_02x, i_03x[0], i_04x, i_05x[0], i_06x, i_07x[0], i_08x, i_09x[0]))
  final_iWall_y = np.concatenate( (i_01y[0], i_02y, i_03y[0], i_04y, i_05y[0], i_06y, i_07y[0], i_08y, i_09y[0]))
  iWallCoord = np.column_stack((final_iWall_x, final_iWall_y)) 
  iWallCoord = iWallCoord.T *0.02

  final_oWall_x = np.concatenate( (o_01x[0], o_02x, o_03x[0], o_04x, o_05x[0], o_06x, o_07x[0], o_08x, o_09x[0]))
  final_oWall_y = np.concatenate( (o_01y[0], o_02y, o_03y[0], o_04y, o_05y[0], o_06y, o_07y[0], o_08y, o_09y[0]))
  oWallCoord = np.column_stack((final_oWall_x, final_oWall_y))
  oWallCoord = oWallCoord.T *0.02


  return pathCoord, iWallCoord, oWallCoord
