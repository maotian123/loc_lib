void fun()
{
    double distance = 10000.0;
    double cos_block_az = FixedPoint2Double(0x4000);
    double sin_block_az = FixedPoint2Double(0x4000);

    double cos_angle_vert = FixedPoint2Double(0x4400);
    double sin_angle_vert = FixedPoint2Double(0x4400);

    double cos_az_azis_channl_diff = FixedPoint2Double(0x4000);
    double sin_az_azis_channl_diff = FixedPoint2Double(0x4000);

    double cos_horiz_angle_channl_diff = FixedPoint2Double(0x4000);
    double sin_horiz_angle_channl_diff = FixedPoint2Double(0x0300);

    
    
    double x = distance * cos_angle_vert * 
    (
        ( 
            cos_block_az * cos_az_azis_channl_diff - 
            sin_block_az *  sin_az_azis_channl_diff 
        ) * 
        cos_horiz_angle_channl_diff -
        (
            sin_block_az * cos_az_azis_channl_diff + 
            cos_block_az *  sin_az_azis_channl_diff 
        ) * 
        sin_horiz_angle_channl_diff
    )


    double y = -distance * cos_angle_vert * 
        (
            (
                sin_block_az * cos_az_azis_channl_diff + 
                cos_block_az *  sin_az_azis_channl_diff 
            ) * 
            cos_horiz_angle_channl_diff + 
            ( 
                cos_block_az * cos_az_azis_channl_diff - 
                sin_block_az *  sin_az_azis_channl_diff 
            ) * 
            sin_horiz_angle_channl_diff  
        ) 

    z = distance * sin_angle_vert + 0
}