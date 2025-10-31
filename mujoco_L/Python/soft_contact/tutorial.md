# Soft Contact
***&emsp;&emsp;软接触由geom中的solimp和solref参数调控***
&emsp;&emsp;虽然geom在mujoco中是刚体，但是通过soft contact可以近似现实中物体碰撞时发生的形变，比如刚性比较强的物体发生的微小变形，又或者是一个可以通过缓冲力的物体，又或者是一个很有弹性的橡皮球，这些在mujoco的刚体中可以通过soft contact近似出这些物体的碰撞情况       
![](../../MJCF/asset/contact.gif)       
![](../../MJCF/asset/soft_solver_param.png)     
[公式计算可视化（desmos）](https://www.desmos.com/calculator/irtgrwjpkb?lang=zh-CN)         
**在这里把碰撞拆解成了下面公式**          
$$a_{ref}=-bv-kr$$      
$$a_{1}=(1-d) \cdot a_{0}-d \cdot a_{ref}$$     
> a1:计算之后的加速度
> a0:无约束时的加速度
> v :速度
> r :陷入深度，两个物体间碰撞会陷入一段距离
> d,b,k:约束参数，solimp和solref计算结果

**这就是一个动态的“弹簧-阻尼”模型，a<sub>ref</sub>是一个“弹簧-阻尼“，陷入深入r越大a<sub>ref</sub>权重越高，约束力越强”**        

## solimp参数       
**solimp这个参数会计算出来上述公式中的d，这个d的计算会和两个物体碰撞时陷入的深度有关，d的范围是(0,1)。下面参数会计算出来d(r)**
> 参数：(d<sub>0</sub>,d<sub>width</sub>,width,midpoint,power)
>- d<sub>0</sub>：d的最小值
>- d<sub>width</sub>：d的最大值
>- width：陷入深度归一化的底数，归一化计算：normal(r)/width
>- midpoint：控制d变化曲线,会使曲线底部变“宽”，曲线分段位置
>- power：控制d变化曲线，会使曲线变化的“更快”

**计算公式**        
$$x_{\text{normal}} = \frac{|r|}{\text{width}}$$        
$$a = \frac{1}{\text{midpoint}^{\text{power}-1}}$$      
$$b = \frac{1}{(1 - \text{midpoint})^{\text{power}-1}}$$        

$$Y(x) = \{
\begin{array}{ll}
a x^{power} & \text{if } x \leq midpoint \\
1 - b (1 - x)^{power} & \text{if } x > midpoint
\end{array}
\}
\quad \text{for} \quad \{ 0 \leq x \leq 1 \}$$      

$$d\left(x_{normal}\right) = d_{0} + Y\left(x_{normal}\right) \left( d_{\text{width}} - d_{0} \right)$$     

**官方文档图像**        
![](../../MJCF/asset/mujoco_doc_solimp.png)         
[**desmos绘制**](https://www.desmos.com/calculator/irtgrwjpkb?lang=zh-CN)       
![](../../MJCF/asset/solimp_img.png)        
**源码位置**        
engine/engine_core_constraint.c:        
static void getimpedance(const mjtNum* solimp, mjtNum pos, mjtNum margin,mjtNum* imp, mjtNum* impP)     
![](../../MJCF/asset/compute_solimp.png)        

## solref参数       
**这个参数影响公式中的k,b**     

> 参数为正值(timeconst,dampratio)
>- timeconst：会影响k,b，数值越小k,b越大，但不会小于两倍的timestep（强制性的）
>- dampratio：会影响k,数值越小k越大，一般设置为1,数值过小会阻尼不够或弹性不足，数值过大会约束过过度

**计算公式**        
$$b=\frac{2}{d_{width} \cdot timeconst}$$       
$$k=\frac{d(r)}{d_{width} \cdot timeconst^2 \cdot dampratio^2}$$        

> 参数为负值(-stiffness,-damping)
>- stiffness：与d<sub>width</sub>一起影响k值
>- damping：与d，d<sub>width</sub>一起影响b值

**计算公式**        
$$b=\frac{damping}{d_{width}}$$     
$$k=\frac{stiffness \cdot d(r)}{d_{width}^2}$$      

[**desmos**](https://www.desmos.com/calculator/irtgrwjpkb?lang=zh-CN)       
![](../../MJCF/asset/solref_desmos.png)     
**源码位置**        
engine/engine_core_constraint.c:        
void mj_makeImpedance(const mjModel* m, mjData* d)      
![](../../MJCF/asset/coompute_solref.png)       
![](../../MJCF/asset/coompute_solref2.png)      

## solimp和solref的混合规则         
> 情况一：根据priority的大小，选择两个碰撞geom中priority大的solimp和solref参数     
> 情况二：如果priority相同则根据两个geom的solmix参数计算一个mix值，为各自solmix占两个solmix和的比例
>* 两个geom的solmix均大于0，mix=mix = solmix1 / (solmix1 + solmix2)
>* 又任意一方的solmix小于等于0,则使用对方的参数
   
源码位置：engine/engine_collision_driver.c: 
mj_contactParam         
![](../../MJCF/asset/mix_con.png)

## 调整思路             
**可以从pd控制器和碰撞曲线两个方面分析碰撞**        
### PD          
$$a_{ref}=-bv-kr$$          
由这个公式可以分析出，如果我们想抑制陷入深度（穿模），那需要增大刚度k的参数，如果碰撞弹性很大或者是接触时抖动剧烈，可能是阻尼b不够      
### 碰撞曲线    
碰撞曲线计算出d参数，会根据陷入深度动态调节pd控制器的比例        

### 例子        
**橡胶材料(RubberBalls)**           
它们的弹性形变量可以很大，那可以调大width参数           
- **如果想要橡胶硬一点**首先增加pd的刚度，其次形变恢复速度要快可以调整曲线让曲线更抖一些(小形变时pd控制器的占比也会比较高)，那就可以增加    d<sub>0</sub>并减小midpoint和power      
- **如果想要橡胶柔一点**，那就是降低刚度，并让曲线缓一点和上述相反      

**金属材料(newton_cradle)**，金属材料的弹性型变量一般比较小，刚性很强，那么width参数就要小一些，刚度k要大一些。然后金属之间碰撞时间会很短，所以碰撞曲线就要变化小，舒缓一些，所以可以d<sub>0</sub>=0,d<sub>width</sub>也要比较小，这样的曲线就很近似短时间碰撞情况      

**缓冲能力较强材料(cushioning)**        
从比较高空落地不怎么反弹，静止时也不会有很大的形变，可以在增加刚度k的同时增加阻尼b      


## 参考
[Computation/Soft contact model](https://mujoco.readthedocs.io/en/latest/computation/index.html#soft-contact-model)
[Modeling/Solver parameters](https://mujoco.readthedocs.io/en/latest/modeling.html#solver-parameters)
