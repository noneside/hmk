# 3D绘制
&emsp;&emsp;mujoco提供显示基础几何体和mujoco提供的一些特殊渲染几何体。查看文档可知mjv_initGeom函数能在渲染场景中增加几何体，mjv_connector可以用mjv_initGeom初始化的几何体绘制提供的一些特殊形状（如箭头，直线等）。mujoco显示画面的原理是通过mjv_updateScene 将仿真数据储存到mjvScene中，这是已经处理好的几何数据，接下来使用mjr_render传递给opengl渲染。我们在绘制过程中是要在仿真的几何数据处理完之后，加入绘制信息，再交给opengl渲染。
&emsp;&emsp;在mjvScene中添加信息，其实是直接在mjvScene的geoms后面续写，而且要增加ngeom长度。这里通过注释可以理解，mjvScene根据ngeom确定几何体数量再从geoms中获取资源。
![](../../MJCF/asset/mjvScene.png)
mjv_initGeom函数原型：
![](../../MJCF/asset/initGeom.png)
mjv_connector函数原型：
![](../../MJCF/asset/mjv_connector.png)
geom是传入的仅绘制的几何体，需要使用mjv_initGeom初始化，type见下面，width是绘制的宽度，这个是对于渲染出来的画面的宽度，from起点，to终点
![](../../MJCF/asset/mjtGeom.png)
这里是可以绘制的几何形状类型，分别是箭头，无楔形箭头，双向箭头，直线。
<font color=Green>*演示——绘制几何体函数：*</font>

```Python
def draw_geom(type, size, pos, mat, rgba):
    viewer.user_scn.ngeom += 1
    geom = viewer.user_scn.geoms[viewer.user_scn.ngeom - 1]   
    mujoco.mjv_initGeom(geom, type, size, pos, mat, rgba)
......
size = [0.0, 0.0, 0.0] 
pos = [0, 0, 0]         
mat = [0, 0, 0, 0, 0, 0, 0, 0, 0] 
rgba = [1.0, 0.0, 0.0, 1.0]     
draw_geom(mujoco.mjtGeom.mjGEOM_SPHERE, size, pos, mat, rgba)
```

<font color=Green>*演示——绘制直线函数：*</font>

```Python
def draw_line(start, end, width, rgba):
  viewer.user_scn.ngeom += 1
  geom = viewer.user_scn.geoms[viewer.user_scn.ngeom - 1]
  size = [0.0, 0.0, 0.0] 
  pos = [0, 0, 0]           
  mat = [0, 0, 0, 0, 0, 0, 0, 0, 0]     
  mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE, size, pos, mat, rgba)
  mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, width, start, end)
......
pos_start = [0, 0, 0]
end = [0, 1, 1]
rgba = [1.0, 0.0, 0.0, 1.0]     
draw_line(pos_start, end, 20, rgba)
```

<font color=Green>*演示——绘制箭头函数：*</font>

```Python
def draw_arrow(start, end, width, rgba):
  viewer.user_scn.ngeom += 1
  geom = viewer.user_scn.geoms[viewer.user_scn.ngeom - 1]
  size = [0.0, 0.0, 0.0] 
  pos = [0, 0, 0]           
  mat = [0, 0, 0, 0, 0, 0, 0, 0, 0]  
  mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE, size, pos, mat, rgba)
  mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_ARROW, width, start, end)
......
pos_start2 = [0, 0, 0]
end2 = [0, -1, 1]
rgba2 = [0.0, 1.0, 0.0, 0.5]
draw_arrow(pos_start2, end2, 0.1, rgba2)
```


# 2D绘制
字体尺寸的初始化：
![](../../MJCF/asset/font_size.png)
python:         
```Python
context = mujoco.MjrContext(m, mujoco.mjtFontScale.mjFONTSCALE_150)
```
查阅文档我们可知2D绘制要在mjr_render之后进行
![](../../MJCF/asset/2D_draw_point.png)

<font color=Green>*绘制文本：*</font>           

```C++
MJAPI void mjr_text(int font, const char* txt, const mjrContext* con,
                    float x, float y, float r, float g, float b);
```
font:字号，使用mjtFont中定义的    
txt：文本   
con：mjrContext   
x,y:渲染界面比例位置，取值[0-1)   
r,g,b:字体颜色    
**Python:**     
```Python
mujoco.mjr_text(mujoco.mjtFont.mjFONT_NORMAL, "Albusgive", context, 0, 0.8, 1, 0, 1)
```

<font color=Green>*绘制对应表格（overlay）：*</font>        

```C++
MJAPI void mjr_overlay(int font, int gridpos, mjrRect viewport,
                       const char* overlay, const char* overlay2, const mjrContext* con);
```
font:字号，使用mjtFont中定义的    
gridpos：绘制位置，使用mjtGridPos中定义的     
mjrRect：mjrRect，界面矩形  
overlay：第一列     
overlay2：第二列      
con：mjrContext      
**Python:**     
```Python
mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT, viewport, "github", "Albusgive", context)
```


<font color=Green>*绘制矩形：*</font>           

```C++
MJAPI void mjr_rectangle(mjrRect viewport, float r, float g, float b, float a);
```
mjrRect：mjrRect，矩形      
rgba:颜色   
**Python:**     
```Python
viewport2 = mujoco.MjrRect(50, 100, 50, 50)
mujoco.mjr_rectangle(viewport2, 0.5, 0, 1, 0.6)
```

<font color=Green>*绘制标签：*</font>           

```C++
MJAPI void mjr_label(mjrRect viewport, int font, const char* txt,
                     float r, float g, float b, float a, float rt, float gt, float bt,
                     const mjrContext* con);
```
viewport：标签位置    
font:字号，使用mjtFont中定义的    
txt：文本   
rgba:标签底色   
rt,gt,bt:文字颜色   
con：mjrContext   
**Python:**     
```Python
viewport3 = mujoco.MjrRect(100, 200, 150, 50)
mujoco.mjr_label(viewport3, mujoco.mjtFont.mjFONT_NORMAL, "Albusgive", 0, 0, 1, 1, 1, 1,1, context)
```
