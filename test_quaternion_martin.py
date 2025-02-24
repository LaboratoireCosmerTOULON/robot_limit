import numpy
import numpy as np
from quaternion import as_float_array, quaternion
from scipy.spatial.transform import Rotation

numpy.set_printoptions( precision = 2, linewidth = 1000 )

import cldLib as cld

current_pose_vector_in_world = np.random.random( (6,) )

linear_speed = np.random.random( (3,) )
angular_speed = np.zeros( (3,) )
time_step = 0.1
print( f"\n{'=' * 20} {'setup':^50} {'=' * 20}\n" )

print( f"{current_pose_vector_in_world=}" )
print( f"{linear_speed=}" )
print( f"{angular_speed=}" )
print( f"{time_step=}" )

print( f"\n{'=' * 20} {'homogeneous current':^50} {'=' * 20}\n" )

homogeneous_matrix_world_to_current = cld.homogeneousMatrix( *current_pose_vector_in_world )
current_translation, current_euler_angles = cld.matrix_to_translation_euler(
    homogeneous_matrix_world_to_current
    )

print( f"{homogeneous_matrix_world_to_current=}" )
print( f"{current_translation=}" )
print( f"{current_euler_angles=}" )

print( f"\n{'=' * 20} {'homogeneous current to next':^50} {'=' * 20}\n" )

homogeneous_matrix_current_to_next = cld.homogeneous_from_twist( angular_speed, linear_speed, time_step )
translation_current_to_next, euler_angles_current_to_next = cld.matrix_to_translation_euler(
    homogeneous_matrix_current_to_next
    )

print( f"{homogeneous_matrix_current_to_next=}" )
print( f"{translation_current_to_next=}" )
print( f"{euler_angles_current_to_next=}" )

print( f"\n{'=' * 20} {'homogeneous next':^50} {'=' * 20}\n" )

homogeneous_world_to_next = homogeneous_matrix_world_to_current @ homogeneous_matrix_current_to_next
next_translation, next_euler_angles = cld.matrix_to_translation_euler( homogeneous_world_to_next )

print( f"{homogeneous_world_to_next=}" )
print( f"{next_translation=}" )
print( f"{next_euler_angles=}" )

print( f"\n{'=' * 20} {'quaternions current':^50} {'=' * 20}\n" )

current_translation, current_quaternion = cld.matrix_to_translation_quaternion( homogeneous_matrix_world_to_current )
print( f"{current_translation=}" )
print( f"{current_quaternion=}" )

print( f"\n{'=' * 20} {'quaternions next':^50} {'=' * 20}\n" )

next_translation, next_quaternion = cld.matrix_to_translation_quaternion( homogeneous_world_to_next )
print( f"{next_translation=}" )
print( f"{next_quaternion=}" )

print( f"\n{'=' * 20} {'quaternions current to next':^50} {'=' * 20}\n" )

# numpy quaternion is [w x y z] and scipy is [x y z w]
quaternion_current_to_next = as_float_array(
    quaternion( next_quaternion[ 3 ], *next_quaternion[ :3 ] ) * quaternion(
        current_quaternion[ 3 ], *current_quaternion[ :3 ]
        ).inverse()
    )
current_to_next_translation = next_translation - current_translation

print( f"{current_to_next_translation=}" )
print( f"{quaternion_current_to_next=}" )

print( f"\n{'=' * 20} {'tests':^50} {'=' * 20}\n" )

current_euler_from_quaternion = Rotation.from_quat(
    current_quaternion, scalar_first = True
    ).as_euler( 'zyx', degrees = True )
euler_current_to_next_from_quaternion = Rotation.from_quat(
    quaternion_current_to_next, scalar_first = True
    ).as_euler( 'zyx', degrees = True )
next_euler_from_quaternion = Rotation.from_quat(
    next_quaternion, scalar_first = True
    ).as_euler( 'zyx', degrees = True )

print( f"{current_quaternion=}" )
print( f"{current_euler_from_quaternion=}" )
print( f"{current_euler_angles=}" )
print( "\n" )
print( f"{quaternion_current_to_next=}" )
print( f"{euler_current_to_next_from_quaternion=}" )
print( f"{euler_angles_current_to_next=}" )
print( "\n" )
print( f"{next_quaternion=}" )
print( f"{next_euler_from_quaternion=}" )
print( f"{next_euler_angles=}" )
print( "\n" )
print( f"{current_quaternion==next_quaternion=}\tshould be all true since {angular_speed=}" )
print( f"{current_euler_from_quaternion==next_euler_from_quaternion=}\tshould be all true since {angular_speed=}" )
print( f"{current_euler_from_quaternion==current_euler_angles=}" )
print( f"{next_euler_from_quaternion==next_euler_angles=}\tshould be all true since {angular_speed=}" )
print( f"{current_euler_angles==next_euler_angles=}\tshould be all true since {angular_speed=}" )

print( f"\n{'=' * 20} {'euler is trash':^50} {'=' * 20}\n" )