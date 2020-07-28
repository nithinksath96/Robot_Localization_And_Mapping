function fusion_map = updateFusionMapWithProjMap(fusion_map, updated_map, h, w, proj_flag)

    %==== TODO: Merge the updated map with the remaining part of the old fusion map ====

    % Write your code here...
      
%     input_points = input_data.pointcloud.Location;
%     input_colors = input_data.pointcloud.Color;
%     input_normals = input_data.normals;
%     proj_points = proj_map.points;
%     proj_colors = proj_map.colors;
%     proj_normals = proj_map.normals;
%     proj_ccounts = proj_map.ccounts;
%     proj_times = proj_map.times;

    unused_flag = not(proj_flag);
    

    unused_points = reshape(fusion_map.pointcloud.Location(repmat(unused_flag, 1, 3)), [], 3);
    unused_colors = reshape(fusion_map.pointcloud.Color(repmat(unused_flag, 1, 3)), [], 3);
    unused_normals = reshape(fusion_map.normals(repmat(unused_flag, 1, 3)), [], 3);
    unused_ccounts = fusion_map.ccounts(unused_flag);
    unused_times = fusion_map.times(unused_flag);
    new_points = reshape(updated_map.points, [h * w, 3]);
    new_colors = reshape(updated_map.colors, [h * w, 3]);
    new_normals = reshape(updated_map.normals, [h * w,3]);
    new_ccounts = reshape(updated_map.ccounts, [h * w, 1]);
    new_times = reshape(updated_map.times, [h * w, 1]);
    

    map_points = cat(1, unused_points, new_points);
    map_colors = cat(1, unused_colors, new_colors);
    map_normals = cat(1, unused_normals, new_normals);
    map_ccounts = cat(1, unused_ccounts, new_ccounts);
    map_times = cat(1, unused_times, new_times);

    map_pointcloud = pointCloud(map_points, 'Color', map_colors);
    fusion_map = struct('pointcloud', map_pointcloud, 'normals', map_normals, 'ccounts', map_ccounts, 'times', map_times);
      
end
   