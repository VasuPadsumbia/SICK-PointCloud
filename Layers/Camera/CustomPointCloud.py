import math
import numpy as np

def convertToPointCloud(distData, intsData, cnfiData, myCamParams, isStereo):
    """
    Return values:
    wCoordinates: Nested List with the linewise data for a pointcloud file. Each list item is a list with the following entries 
                  X Y Z R G B I   i.e. point coordinates (XYZ), color (RGB) and intensity (I)
    distData: input distData reshaped to array with camera resolution
    """
    wCoordinates = []

    m_c2w = np.array(myCamParams.cam2worldMatrix)
    shape = (4, 4)
    m_c2w.shape = shape

    cnfiData = np.asarray(cnfiData).reshape(
        myCamParams.height, myCamParams.width)
    intsData = np.asarray(intsData).reshape(
        myCamParams.height, myCamParams.width)
    distData = np.asarray(list(distData)).reshape(
        myCamParams.height, myCamParams.width)

    if isStereo:

        # RGBA intensities
        intsData = np.asarray(intsData).astype('uint32').view(
            'uint8').reshape(myCamParams.height, myCamParams.width, 4)
        intsData = np.frombuffer(intsData, np.uint8).reshape(
            myCamParams.height, myCamParams.width, 4)
        color_map = intsData

        # Apply the Statemap to the Z-map
        zmapData_with_statemap = np.array(distData).reshape(
            myCamParams.height, myCamParams.width)

        for row in range(myCamParams.height):
            for col in range(myCamParams.width):
                if (cnfiData[row][col] != 0):
                    # Set invalid pixels to lowest value
                    zmapData_with_statemap[row][col] = 0
                else:
                    # use all "good" points to export to PLY

                    # transform into camera coordinates (zc, xc, yc)
                    xp = (myCamParams.cx - col) / myCamParams.fx
                    yp = (myCamParams.cy - row) / myCamParams.fy

                    # coordinate system local to the imager
                    zc = distData[row][col]
                    xc = xp * zc
                    yc = yp * zc

                    # convert to world coordinate system
                    xw = (m_c2w[0, 3] + zc * m_c2w[0, 2] +
                          yc * m_c2w[0, 1] + xc * m_c2w[0, 0])
                    yw = (m_c2w[1, 3] + zc * m_c2w[1, 2] +
                          yc * m_c2w[1, 1] + xc * m_c2w[1, 0])
                    zw = (m_c2w[2, 3] + zc * m_c2w[2, 2] +
                          yc * m_c2w[2, 1] + xc * m_c2w[2, 0])

                    # merge 3D coordinates and color
                    wCoordinates.append(
                        [xw, yw, zw, color_map[row][col][0], color_map[row][col][1], color_map[row][col][2], 0])

        return wCoordinates, distData

    else:

        for row in range(0, myCamParams.height):
            for col in range(0, myCamParams.width):

                # calculate radial distortion
                xp = (myCamParams.cx - col) / myCamParams.fx
                yp = (myCamParams.cy - row) / myCamParams.fy

                r2 = (xp * xp + yp * yp)
                r4 = r2 * r2

                k = 1 + myCamParams.k1 * r2 + myCamParams.k2 * r4

                xd = xp * k
                yd = yp * k

                d = distData[row][col]
                s0 = np.sqrt(xd*xd + yd*yd + 1)

                xc = xd * d / s0
                yc = yd * d / s0
                zc = d / s0 - myCamParams.f2rc

                # convert to world coordinate system
                xw = (m_c2w[0, 3] + zc * m_c2w[0, 2] +
                      yc * m_c2w[0, 1] + xc * m_c2w[0, 0])
                yw = (m_c2w[1, 3] + zc * m_c2w[1, 2] +
                      yc * m_c2w[1, 1] + xc * m_c2w[1, 0])
                zw = (m_c2w[2, 3] + zc * m_c2w[2, 2] +
                      yc * m_c2w[2, 1] + xc * m_c2w[2, 0])

                # convert to full decibel values * 0.01, which is the same format that Sopas uses for point cloud export
                intsSopasFormat = round(
                    0.2 * math.log10(intsData[row][col]), 2) if intsData[row][col] > 0 else 0

                # merge 3D coordinates and intensity
                wCoordinates.append([xw, yw, zw, 0, 0, 0, intsSopasFormat])

        return wCoordinates, distData
    
def convertToPointCloudOptimized(distData: list, cnfiData: list, myCamParams: list, isStereo: bool, 
                                 depth_range: tuple = (500, 1000), roi: tuple = None):
    """
    Converts 2D image data to a 3D point cloud, isolating a specific object (e.g., cardboard).

    Parameters:
    distData (list): The distance data from the sensor.
    cnfiData (list): The confidence data from the sensor.
    myCamParams (list): The camera parameters.
    isStereo (bool): Whether the camera is stereo.
    depth_range (tuple): Depth range (min, max) in millimeters for isolating the object.
    roi (tuple): Region of interest (x_start, y_start, width, height) in pixel coordinates.

    Returns:
    numpy.ndarray: Filtered 3D point cloud of the cardboard.
    """
    SENSOR_TO_ORIGIN_DIST = myCamParams.cam2worldMatrix[11]

    # Camera to world transformation matrix
    m_c2w = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, SENSOR_TO_ORIGIN_DIST],
        [0, 0, 0, 1]
    ])

    m_c2w.shape = (4, 4)
    distData = np.asarray(list(distData)).reshape(myCamParams.height, myCamParams.width)
    cnfiData = np.asarray(cnfiData).reshape(myCamParams.height, myCamParams.width)

    if roi:
        x_start, y_start, roi_width, roi_height = roi
        distData = distData[y_start:y_start + roi_height, x_start:x_start + roi_width]
        cnfiData = cnfiData[y_start:y_start + roi_height, x_start:x_start + roi_width]

        # Generate point cloud
        cols = np.arange(0, distData.shape[1])
        rows = np.arange(0, distData.shape[0])
        xp = (cols - myCamParams.cx) / myCamParams.fx
        yp = (rows - myCamParams.cy) / myCamParams.fy
        xp = xp[:, np.newaxis]
        yp = yp[np.newaxis, :]

        xc = distData * xp.T
        yc = yp.T * distData
        zc = distData

        xw = (m_c2w[0, 3] + zc * m_c2w[0, 2] +
              yc * m_c2w[0, 1] + xc * m_c2w[0, 0])
        yw = (m_c2w[1, 3] + zc * m_c2w[1, 2] +
              yc * m_c2w[1, 1] + xc * m_c2w[1, 0])
        zw = (m_c2w[2, 3] + zc * m_c2w[2, 2] +
              yc * m_c2w[2, 1] + xc * m_c2w[2, 0])

        point_cloud = np.stack([xw, yw, zw], axis=-1)

        # Filter by depth range
        depth_min, depth_max = depth_range
        depth_mask = (distData >= depth_min) & (distData <= depth_max)

        # Filter by confidence
        confidence_mask = cnfiData > 0  # Adjust threshold if needed

        # Combine masks
        mask = depth_mask & confidence_mask
        filtered_cloud = point_cloud[mask]
        return filtered_cloud
    else:
        # return wCoordinates
        cols = np.arange(0, myCamParams.width)
        rows = np.arange(0, myCamParams.height)
        xp = (cols - myCamParams.cx) / myCamParams.fx
        yp = (rows - myCamParams.cy) / myCamParams.fy
        xp = xp[:, np.newaxis]
        yp = yp[np.newaxis, :]

        xc = distData * xp.T
        yc = yp.T * distData

        zc = distData

        xw = (m_c2w[0, 3] + zc * m_c2w[0, 2] +
              yc * m_c2w[0, 1] + xc * m_c2w[0, 0])
        yw = (m_c2w[1, 3] + zc * m_c2w[1, 2] +
              yc * m_c2w[1, 1] + xc * m_c2w[1, 0])
        zw = (m_c2w[2, 3] + zc * m_c2w[2, 2] +
              yc * m_c2w[2, 1] + xc * m_c2w[2, 0])

        wCoordinates = np.stack([xw, yw, zw], axis=-1)
        cloud_data_like_sopas = wCoordinates.reshape(
            (myCamParams.height, myCamParams.width, 3))
        '''
        offset the entire cordinate system so that the middle pixel is at (0,0,z) ->
        i want to define the system so that the technical drawing, point 7 in here: https://www.sick.com/il/en/catalog/products/machine-vision-and-identification/machine-vision/visionary-t-mini/v3s105-1aaaaaa/p/p665983?tab=detail
        is at (0,0,z) in the world cordinate system.
        that point is defined in pixel coordinates as: (myCamParams.width // 2, myCamParams.height // 2). 
        so range_data[myCamParams.height // 2, myCamParams.width // 2] is the distance from sensor of that point.
        and cloud_data[myCamParams.height // 2, myCamParams.width // 2,:] is the 3D coordinates of that point.
        '''
        cloud_data = cloud_data_like_sopas.copy()
        sensor_center_x_y_values = (cloud_data_like_sopas[myCamParams.height // 2, myCamParams.width //
                                    2, :2] + cloud_data_like_sopas[(-1+myCamParams.height) // 2, (-1+myCamParams.width) // 2, :2])/2
        # making the middle pixel be the origin of the coordinates system (x=0,y=0) and each point is relative to it
        cloud_data[:, :, :2] = cloud_data_like_sopas[:, :, :2] - \
            sensor_center_x_y_values
        # where cnfiData is 0, so set it to a numpy array of 0,0,0
        cloud_data[cnfiData != 0] = np.array([0, 0, 0])

        return cloud_data
    
def convertToPointCloudROI(distData, intsData, cnfiData, myCamParams, isStereo, roi):
    """
    Generate a point cloud within a specified rectangular region.
    
    Args:
        distData: Depth data.
        intsData: Intensity data.
        cnfiData: Confidence data.
        myCamParams: Camera parameters.
        isStereo: Boolean flag for stereo mode.
        roi: Tuple (x_min, x_max, y_min, y_max) defining the rectangle in image coordinates.
    
    Returns:
        wCoordinates: Filtered point cloud within the rectangle.
        distData: Reshaped distance data.
    """
    wCoordinates = []
    m_c2w = np.array(myCamParams.cam2worldMatrix).reshape((4, 4))

    # Reshape data
    cnfiData = np.asarray(cnfiData).reshape(myCamParams.height, myCamParams.width)
    intsData = np.asarray(intsData).reshape(myCamParams.height, myCamParams.width)
    distData = np.asarray(list(distData)).reshape(myCamParams.height, myCamParams.width)
    # Apply the Statemap to the Z-map
    zmapData_with_statemap = np.array(distData).reshape(
            myCamParams.height, myCamParams.width) 
     
    x_min, x_max, y_min, y_max = roi  # Extract ROI boundaries

    for row in range(y_min, min(y_max, myCamParams.height)):
        for col in range(x_min, min(x_max, myCamParams.width)):
            if cnfiData[row][col] != 0:
                zmapData_with_statemap[row][col] = 0
                
            else:

                yp = (myCamParams.cy - row) / myCamParams.fy
                xp = (myCamParams.cx - col) / myCamParams.fx

                zc = distData[row][col]
                xc = xp * zc
                yc = yp * zc

                # Convert to world coordinates
                xw = (m_c2w[0, 3] + zc * m_c2w[0, 2] + yc * m_c2w[0, 1] + xc * m_c2w[0, 0])
                yw = (m_c2w[1, 3] + zc * m_c2w[1, 2] + yc * m_c2w[1, 1] + xc * m_c2w[1, 0])
                zw = (m_c2w[2, 3] + zc * m_c2w[2, 2] + yc * m_c2w[2, 1] + xc * m_c2w[2, 0])

                if isStereo:
                     # RGBA intensities
                    intsData = np.asarray(intsData).astype('uint32').view(
                        'uint8').reshape(myCamParams.height, myCamParams.width, 4)
                    intsData = np.frombuffer(intsData, np.uint8).reshape(
                        myCamParams.height, myCamParams.width, 4)
                    color_map = intsData
                    wCoordinates.append([xw, yw, zw, color_map[row][col][0], color_map[row][col][1], color_map[row][col][2], 0])
                else:
                    intensity = round(0.2 * math.log10(intsData[row][col]), 2) if intsData[row][col] > 0 else 0
                    wCoordinates.append([xw, yw, zw, 0, 0, 0, intensity])

    return wCoordinates, distData
