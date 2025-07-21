import numpy as np
import matplotlib.pyplot as plt

def simulate_magnetic_field(tilt_angle_deg=0):
    """
    模拟地磁场在不同倾斜角度下的X、Y分量
    
    Args:
        tilt_angle_deg: 传感器倾斜角度（度）
    """
    # 地磁场参数
    B_total = 50000  # 地磁场总强度 (nT)
    magnetic_declination = 0  # 磁偏角
    magnetic_inclination = 60  # 磁倾角（北半球典型值）
    
    # 转换为弧度
    tilt_rad = np.radians(tilt_angle_deg)
    decl_rad = np.radians(magnetic_declination)
    incl_rad = np.radians(magnetic_inclination)
    
    # 生成旋转角度（0到360度）
    rotation_angles = np.linspace(0, 2*np.pi, 360)
    
    # 计算地磁场分量
    B_x = []
    B_y = []
    
    for phi in rotation_angles:
        # 地磁场在水平面的分量
        B_horizontal = B_total * np.cos(incl_rad)
        
        # 考虑传感器倾斜的影响
        # 倾斜会改变X、Y分量的比例
        B_x_comp = B_horizontal * np.cos(phi) * np.cos(tilt_rad)
        B_y_comp = B_horizontal * np.sin(phi) * np.cos(tilt_rad)
        
        B_x.append(B_x_comp)
        B_y.append(B_y_comp)
    
    return np.array(B_x), np.array(B_y)

def plot_tilt_comparison():
    """绘制不同倾斜角度的对比图"""
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    # 测试不同的倾斜角度
    tilt_angles = [0, 15, 30, 45]
    
    for i, tilt_angle in enumerate(tilt_angles):
        ax = axes[i//2, i%2]
        
        # 模拟数据
        B_x, B_y = simulate_magnetic_field(tilt_angle)
        
        # 绘制
        ax.plot(B_x, B_y, 'b-', linewidth=2, alpha=0.7)
        ax.plot(B_x, B_y, 'r.', markersize=3)
        ax.plot(0, 0, 'ko', markersize=8)
        
        # 设置标题和标签
        ax.set_title(f'传感器倾斜 {tilt_angle}°\n地磁场X-Y投影')
        ax.set_xlabel('磁力计X轴 (nT)')
        ax.set_ylabel('磁力计Y轴 (nT)')
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        # 计算椭圆度
        x_range = B_x.max() - B_x.min()
        y_range = B_y.max() - B_y.min()
        ellipticity = max(x_range, y_range) / min(x_range, y_range) if min(x_range, y_range) > 0 else 1
        
        ax.text(0.02, 0.98, f'椭圆度: {ellipticity:.3f}', 
                transform=ax.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    plt.show()

def analyze_real_data():
    """分析真实数据的椭圆度"""
    print("=== 地磁场倾斜影响分析 ===")
    print("倾斜角度 | 椭圆度 | 形状描述")
    print("-" * 30)
    
    for tilt in [0, 5, 10, 15, 20, 30, 45]:
        B_x, B_y = simulate_magnetic_field(tilt)
        x_range = B_x.max() - B_x.min()
        y_range = B_y.max() - B_y.min()
        ellipticity = max(x_range, y_range) / min(x_range, y_range) if min(x_range, y_range) > 0 else 1
        
        shape_desc = "圆形" if ellipticity < 1.05 else f"椭圆(长轴/短轴={ellipticity:.2f})"
        print(f"{tilt:8d}° | {ellipticity:6.3f} | {shape_desc}")

if __name__ == "__main__":
    print("地磁场倾斜对磁力计X、Y数据的影响分析")
    print("=" * 50)
    
    # 分析不同倾斜角度的影响
    analyze_real_data()
    
    # 绘制对比图
    plot_tilt_comparison()
    
    print("\n结论：")
    print("1. 传感器水平时，X-Y数据形成圆形")
    print("2. 传感器倾斜时，X-Y数据形成椭圆")
    print("3. 倾斜角度越大，椭圆度越高")
    print("4. 这就是为什么需要倾斜补偿的原因") 