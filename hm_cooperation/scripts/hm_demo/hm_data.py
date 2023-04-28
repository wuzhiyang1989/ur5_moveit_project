import rospy

# init_joint
init_joint = [-0.0318784608867538, -1.1706948260942116, -0.02358112030386411, 
              -2.61693128920438  , 0.0002019910953364892, 1.5272900696595506, 0.779742430658804   ]

# object_joint_01 = [joint_01, joint_02, joint_03, joint_04, joint_05, joint_06, joint_07]
# glasses box
object_joint_00  = [-1, -1 -1, -1, -1, -1, -1]
prepare_joint_00 = [-1, -1 -1, -1, -1, -1, -1]

object_joint_01  = [0.49521473763297347, 0.33863727397485277, 0.16142387945401038,
                    -0.9083622781435648, 1.2638523199390985, 2.2625359809332424, -0.4023414184538028]
prepare_joint_01 = [0.2741026195517757, 0.19953159374549814, 0.1792872863472473,
                    -1.0466103503317676, 1.3383400332331656, 1.9909382633301946, -0.37779732603703936]

object_joint_02  = [-0.16817016166553161, -0.02534910847339933, 0.880977014592613,
                    -1.3398145485760873, 1.4200852112543516, 2.3123650189174545, -0.5238747535037498]
prepare_joint_02 = [-0.4918401788201248, -0.35803512075909333, 0.579224415739559,
                    -1.562581670995344, 1.7415348946398472, 1.7279639279758257, -0.49146895937517576]

object_joint_03  = [-1, -1 -1, -1, -1, -1, -1]
prepare_joint_03 = [-1, -1 -1, -1, -1, -1, -1]

object_joint_04  = [0.3193069255259353, 0.5172704872587055, 0.23253695611995562,
                    -1.2838629536043131, 1.5741252549024463, 2.085797969890965, -0.9585416064611274]
prepare_joint_04 = [0.10119310623721073, 0.36620056985598387, 0.22921840652666656,
                    -1.4716903943346258, 1.5530675668888643, 1.877213263902399, -0.9814170290910535]


object_joint_05  = [0.31563955590179676, 0.19397061124675408, 0.3182561479537422,
                    -1.6944697065520704, 1.588046922488345, 2.2260645313991443, -1.0257364819855095]
prepare_joint_05 = [-0.010922863851568647, -0.015177613790314286, 0.3127998982605181,
                    -1.9671192666103963, 1.6512619864344598, 1.8466292321615747, -1.1172394296106347]

object_joint_06  = [-1, -1 -1, -1, -1, -1, -1]
prepare_joint_06 = [-1, -1 -1, -1, -1, -1, -1]

object_joint_07  = [-0.08705955462898538, -0.5223497442982227, 0.8192528289635826,
                    -2.2918337094156365, 2.1398226900804627, 2.072068343553278, -1.109964149980495]
prepare_joint_07 = [-0.13454353428723517, -0.8298032965116333, 0.48927490220284164,
                    -2.5974771185088574, 1.938349069781264, 1.7218033158712918, -1.0156918368625145]

object_joint_08  = [-1, -1 -1, -1, -1, -1, -1]
prepare_joint_08 = [-1, -1 -1, -1, -1, -1, -1]

object_joint_09  = [0.33742445864308895, 0.5367396311676293, 0.27738697997972517,
                    -1.862181781295163, 1.8899347149696613, 2.0625259323789042, -1.6587921873800047]
# prepare_joint_09 = [0.14954855090693422, 0.44618509278799356, 0.30817478888494926,
#                     -2.03632444201493, 1.7504486071434286, 2.0378551089111956, -1.662594391266421]
prepare_joint_09 = [0.2094466984590498, 0.4783064002187328, 0.36494075566340944,
                    -1.9327324475220542, 1.8359186732139852, 2.08102782765362, -1.6178729313512643]

object_joint_10  = [-1, -1 -1, -1, -1, -1, -1]
prepare_joint_10 = [-1, -1 -1, -1, -1, -1, -1]

object_joint_11  = [0.2692858778784733, 0.16550733655856698, 0.6040207235269379,
                    -2.43006436123818, 2.2784377592413594, 2.1317777367301196, -1.961033277698689]
prepare_joint_11 = [0.16717708586087002, -0.04522191536478829, 0.5988539412398085,
                    -2.596647266322618, 2.2165084513331386, 2.0326465467479493, -1.9224637712031725]

# 分拣区
sort_position_01 = [0.31684967060605310, -0.1053309046147575]
sort_position_02 = [0.32703576253174016, 0.13147325958372055]
sort_position_03 = [0.31835155895860984, 0.35375395549890415]
sort_position_04 = [0.60741633033361510, -0.0884076818426018]
sort_position_05 = [0.61053557812750410, 0.12972539314085857]
sort_position_06 = [0.58322681186169530, 0.34382503274417847]

# 订单区
trash_pose        = [0.24704657735182123, -0.3262370965437462]
order_position_01 = [0.40647480422522675, -0.3420465766849094]
order_position_02 = [0.54024019938067690, -0.3461866426316133]

# 回收站
order_position_03 = [0.69444337870764880, -0.3469786774083191]

# put orientation
# orientation_1 = [0.9245488995899517, -0.38069091926405424, 0.0004817991103158493, 0.01683817467887328]
# orientation_1 = [0.94281022, -0.33333, 0, 0]
orientation_1 = [-0.9160100865839683, 0.39893215826142636, -0.03405762996581623, 0.024874328582348594]
# orientation_1 = [1, 0, 0, 0]
# import array
object_joint = [object_joint_00, object_joint_01, object_joint_02, object_joint_03,
                object_joint_04, object_joint_05, object_joint_06, object_joint_07,
                object_joint_08, object_joint_09, object_joint_10, object_joint_11]

prepare_joint = [prepare_joint_00, prepare_joint_01, prepare_joint_02, prepare_joint_03,
                 prepare_joint_04, prepare_joint_05, prepare_joint_06, prepare_joint_07,
                 prepare_joint_08, prepare_joint_09, prepare_joint_10, prepare_joint_11]

sort_position = [sort_position_01, sort_position_02, sort_position_03,
                 sort_position_04, sort_position_05, sort_position_06]

order_position = [order_position_01, order_position_02, order_position_03, trash_pose]

product_height = [0.032, 0.032, 0.046, -1, 0.040, 0.127, -1, 0.115, -1, 0.068, -1, 0.05]

product_topic = ["/aruco_simple/pose_601", "/aruco_simple/pose_602",
                 "/aruco_simple/pose_603", "/aruco_simple/pose_604",
                 "/aruco_simple/pose_605", "/aruco_simple/pose_606",
                 "/aruco_simple/pose_607", "/aruco_simple/pose_608",
                 "/aruco_simple/pose_609", "/aruco_simple/pose_610",
                 "/aruco_simple/pose_611", "/aruco_simple/pose_612"] 
def test():
    print("hm_data impost succeed!")


'''
    sort 1
     x: 0.3168496706060531
        y: -0.1053309046147575
        z: 0.3864546121371938
    sort 2
    x: 0.32703576253174016
        y: 0.13147325958372055
        z: 0.3711373548911687

    sort 3
    x: 0.31835155895860984
        y: 0.35375395549890415
        z: 0.41917689642554357

    sort 4
     x: 0.6074163303336151
        y: -0.08840768184260181
        z: 0.33111437473581823

    sort 5
     x: 0.6105355781275041
        y: 0.12972539314085857
        z: 0.3419239672083013

    sort 6
     x: 0.5832268118616953
        y: 0.34382503274417847
        z: 0.40393152816986777

    order  1
    x: 0.24704657735182123
        y: -0.3262370965437462
        z: 0.37649236856825474

    order 2
      position: 
        x: 0.40647480422522675
        y: -0.3420465766849094
        z: 0.38785269259676547

    order 3
     x: 0.5402401993806769
        y: -0.3461866426316133
        z: 0.3810784208774776

    trash_pose
    x: 0.6944433787076488
        y: -0.34697867740831917
        z: 0.34331739270976436


    standerd  orientation0: 
        x: 0.379019601622865
        y: 0.9253883764598616
        z: -0.0006634930780937908
        w: 0.0002325415265594477

    standerd  orientation0: 
        x: 0.9245488995899517
        y: -0.38069091926405424
        z: 0.0004817991103158493
        w: 0.01683817467887328

    temp orientation:
        x: -0.9160100865839683
        y: 0.39893215826142636
        z: -0.03405762996581623
        w: 0.024874328582348594

'''