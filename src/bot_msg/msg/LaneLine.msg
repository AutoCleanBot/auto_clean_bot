
# 车道线信息
uint8 line_tyep # 车道线类型,对应 lane_detector中的 LaneLineType
int8 pos_type # 车道线位置关系,对应 lane_detector中的 LaneLinePositionType

LaneLineCubicCurve curve_car_coord # 车道线在车辆坐标系下的曲线
LaneLineCubicCurve curve_camera_coord # 车道线在相机系下的曲线
LaneLineCubicCurve curve_img_coord # 车道线在图像坐标系下的曲线

Point2DF[] curve_image_point_set # 车道线在图像坐标系下的点集
Point3DF[] curve_camera_point_set # 车道线在相机坐标系下的点集
Point2DF[] curve_car_point_set # 车道线在车辆坐标系下的点集

EndPoints image_end_point_set # 车道线在图像坐标系下的端点集
uint16 track_id # 车道线的track id
float32 confidence # 车道线的置信度
# Definition of the use type of a lane mark in lane adjustment
uint8 use_type # REAL = 0，VIRTUAL = 1


