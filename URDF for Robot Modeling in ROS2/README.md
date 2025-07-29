# URDF
URDF (Unified Robot Description Format) é um formato de arquivo em XML usado no ROS para descrever a estrutura física de um robô — como seus links (partes rígidas), juntas (articulações), sensores e outras propriedades.

## O que o URDF descreve?
1. Links
    * As partes físicas do robô (base, rodas, braços, sensores etc.)
    * Cada link tem: Massa, Inércia, Geometria (caixa, cilindro, malha STL, etc.), Cor e material

2. Joints
    * As conexões entre os links (ex: uma roda presa ao chassi)
    * Tipos de juntas: fixed (fixa), revolute (rotacional com limite), continuous (rotacional sem limite), prismatic (linear), floating, planar (menos comuns)

3. Transmissions (opcional)
    * Usadas em conjunto com ros_control, para mapear motores para juntas.

4. Plugins e sensores (opcional)
    * Câmeras, LiDARs, IMUs etc., geralmente com uso de extensões como o Gazebo.