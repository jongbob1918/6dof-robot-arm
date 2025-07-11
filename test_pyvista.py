import pyvista as pv

# 간단한 구(Sphere) 메쉬를 만듭니다.
sphere = pv.Sphere()

# Plotter 객체를 생성하고 메쉬를 추가합니다.
plotter = pv.Plotter()
plotter.add_mesh(sphere, color='tan', show_edges=True)

# 카메라 위치를 설정하고 창을 보여줍니다.
plotter.camera_position = 'xy'
print("PyVista 테스트 창을 엽니다...")
plotter.show()
print("테스트 창을 닫았습니다.")