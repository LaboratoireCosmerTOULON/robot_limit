import numpy
import numpy as np
from quaternion import as_float_array, quaternion

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
translation_current, euler_angles_current = cld.matrix_to_translation_euler(
    homogeneous_matrix_world_to_current
    )

print( f"{homogeneous_matrix_world_to_current=}" )
print( f"{translation_current=}" )
print( f"{euler_angles_current=}" )

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
translation_next, euler_angles_next = cld.matrix_to_translation_euler( homogeneous_world_to_next )

print( f"{homogeneous_world_to_next=}" )
print( f"{translation_next=}" )
print( f"{euler_angles_next=}" )

print( f"\n{'=' * 20} {'quaternions current':^50} {'=' * 20}\n" )

translation_current, quaternion_current = cld.matrix_to_translation_quaternion( homogeneous_matrix_world_to_current )
print( f"{translation_current=}" )
print( f"{quaternion_current=}" )

print( f"\n{'=' * 20} {'quaternions next':^50} {'=' * 20}\n" )

translation_next, quaternion_next = cld.matrix_to_translation_quaternion( homogeneous_world_to_next )
print( f"{translation_next=}" )
print( f"{quaternion_next=}" )

print( f"\n{'=' * 20} {'quaternions current to next':^50} {'=' * 20}\n" )

# numpy quaternion is [w x y z] and scipy is [x y z w]
quaternion_current_to_next = as_float_array(
    quaternion( quaternion_next[ 3 ], *quaternion_next[ :3 ] ) * quaternion(
        quaternion_current[ 3 ], *quaternion_current[ :3 ]
        ).inverse()
    )
quaternion_current_to_next = np.array(
    [ quaternion_current_to_next[ 1 ], quaternion_current_to_next[ 2 ], quaternion_current_to_next[ 3 ],
      quaternion_current_to_next[ 0 ] ]
    )
translation_current_to_next = translation_next - translation_current

print( f"{translation_current_to_next=}" )
print( f"{quaternion_current_to_next=}" )

print( f"\n{'=' * 20} {'tests':^50} {'=' * 20}\n" )

euler_from_quaternion_current = cld.quaternion_to_euler( quaternion_current )
euler_from_quaternion_current_to_next = cld.quaternion_to_euler( quaternion_current_to_next )
euler_from_quaternion_next = cld.quaternion_to_euler( quaternion_next )

print( f"{quaternion_current=}" )
print( f"{euler_from_quaternion_current=}" )
print( f"{euler_angles_current=}" )
print( "\n" )
print( f"{quaternion_current_to_next=}" )
print( f"{euler_from_quaternion_current_to_next=}" )
print( f"{euler_angles_current_to_next=}" )
print( "\n" )
print( f"{quaternion_next=}" )
print( f"{euler_from_quaternion_next=}" )
print( f"{euler_angles_next=}" )
print( "\n" )
print(
    f"{'\033[0;42m' if np.all( quaternion_current == quaternion_next ) else '\033[0;41m'}"
    f"{quaternion_current==quaternion_next=}\033[0m"
    f"\tshould be all true since {angular_speed=}"
    )
print(
    f"{'\033[0;42m' if np.all( euler_from_quaternion_current == euler_from_quaternion_next ) else '\033[0;41m'}"
    f"{euler_from_quaternion_current==euler_from_quaternion_next=}\033[0m"
    f"\tshould be all true since {angular_speed=}"
    )
print(
    f"{'\033[0;42m' if np.all( euler_from_quaternion_current == euler_angles_current ) else '\033[0;41m'}"
    f"{euler_from_quaternion_current==euler_angles_current=}\033[0m"
    f"\tshould be all true since the quaternion was made with the same matrix as the angles"
    )
print(
    f"{'\033[0;42m' if np.all( euler_from_quaternion_next == euler_angles_next ) else '\033[0;41m'}"
    f"{euler_from_quaternion_next==euler_angles_next=}\033[0m"
    f"\tshould be all true since {angular_speed=}"
    )
print(
    f"{'\033[0;42m' if np.all( euler_angles_current == euler_angles_next ) else '\033[0;41m'}"
    f"{euler_angles_current==euler_angles_next=}\033[0m"
    f"\tshould be all true since {angular_speed=}"
    )
