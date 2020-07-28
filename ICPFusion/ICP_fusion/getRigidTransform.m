function [tform valid_pair_num error] = getRigidTransform(new_pointcloud, ref_pointcloud, ref_normals)
    
    %==== Initialize parameters ====
    iter_num = 6;
    d_th = 0.05;
    m = size(new_pointcloud.Location, 1);
    n = size(new_pointcloud.Location, 2);
    tform = affine3d(eye(4));
    
    %==== Main iteration loop ====
    for iter = 1:iter_num
        
        %==== Set variables ====
        new_pts = new_pointcloud.Location;
        ref_pts = ref_pointcloud.Location;
        
        %==== For each reference point, find the closest new point within a local patch of size 3-by-3 ====        
        %==== (Notice: assoc_pts[] has the same size and format as new_pts[] and ref_pts[]) ====
        %==== (Notice: assoc_pts[i, j, :] = [0 0 0] iff no point in new_pts[] matches to ref_pts[i, j, :]) ====
        assoc_pts = findLocalClosest(new_pts, ref_pts, m, n, d_th);
        
        %==== Set the sizes of matrix A[] and vertor b[] of normal equation: A'*A*x = A'*b ====
        A = zeros(m*n, 6);
        b = zeros(m*n, 1);
        
        %==== declare the number of point pairs that are used in this iteration ==== 
        valid_pair_num = 0;
        
        %==== TODO: Assign values to A[] and b[] ====
        %==== (Notice: the format of the desired 6-vector is: xi = [beta gamma alpha t_x t_y t_z]') ====

        % Write your code here...
        
%         ref_ptsx = ref_pts(:,:,1);
%         ref_ptsy = ref_pts(:,:,2);
%         ref_ptsz = ref_pts(:,:,3);
%         
%         assoc_ptsx = assoc_pts(:,:,1);
%         assoc_ptsy = assoc_pts(:,:,2);
%         assoc_ptsz = assoc_pts(:,:,3);
%         
%         
%         zerosRowIdx = find(ismember(Vn, [0 0 0],'rows'));
%         
%         Vn(zerosRowIdx, :) = [];
%         Vr(zerosRowIdx, :) = [];
%         Nr(zerosRowIdx, :) = [];
%         
% 
%         
%         refn_x = ref_normals(:,:,1); 
%         refn_y = ref_normals(:,:,2);
%         refn_z = ref_normals(:,:,3);
%         
%         A1=refn_z(:).*assoc_ptsy(:) - refn_y(:).*assoc_ptsz(:);
%         A2=refn_x(:).*assoc_ptsz(:) - refn_z(:).*assoc_ptsx(:);
%         A3=refn_y(:).*assoc_ptsx(:) - refn_x(:).*assoc_ptsy(:);
% %         
%         A4=refn_x(:);
%         A5=refn_y(:);
%         A6=refn_z(:);
% %         
%         A=[-A1,-A2,-A3,A4,A5,A6];
%         
%         b= refn_x(:).*ref_ptsx(:) +refn_y(:).*ref_ptsy(:) +refn_z(:).*ref_ptsz(:) -refn_x(:).*assoc_ptsx(:) -refn_y(:).*assoc_ptsy(:) -refn_z(:).*assoc_ptsz(:);
%         
        
        ref_pts = reshape(ref_pts, [], 3);
        assoc_pts = reshape(assoc_pts, [], 3);
        ref_norm = reshape(ref_normals, [], 3);

        
        zerosRowIdx = find(ismember(assoc_pts, [0 0 0],'rows'));
        
        assoc_pts(zerosRowIdx, :) = [];
        ref_pts(zerosRowIdx, :) = [];
        ref_norm(zerosRowIdx, :) = [];
        
        valid_pair_num = size(assoc_pts,1);
        
        a1 = ref_norm(:,3).*assoc_pts(:,2) - ref_norm(:,2).*assoc_pts(:,3);
        a2 = ref_norm(:,1).*assoc_pts(:,3) - ref_norm(:,3).*assoc_pts(:,1);
        a3 = ref_norm(:,2).*assoc_pts(:,1) - ref_norm(:,1).*assoc_pts(:,2);
        
%         A = [a2 a3 a1 Nr(:,1) Nr(:,2) Nr(:,3)];        
%         A = [a1 a2 a3 Nr(:,1) Nr(:,2) Nr(:,3)];
        A = [-a1 -a2 -a3 ref_norm(:,1) ref_norm(:,2) ref_norm(:,3)];     

        b = ref_norm(:,1).*ref_pts(:,1) + ref_norm(:,2).*ref_pts(:,2) + ref_norm(:,3).*ref_pts(:,3) - ref_norm(:,1).*assoc_pts(:,1) - ref_norm(:,2).*assoc_pts(:,2) - ref_norm(:,3).*assoc_pts(:,3);

        
        %==== TODO: Solve for the 6-vector xi[] of rigid body transformation ====

        % Write your code here...
        
        
        
        R=chol(A'*A);
        y=forward_sub(R',A'*b);
        xi=back_sub(R,y);


        

        %==== Coerce xi[] back into SE(3) ====
        %==== (Notice: tmp_tform[] is defined in the format of right-multiplication) ====
        R = toSkewSym(xi(1:3)) + eye(3);
        [U,S,V] = svd(R);
        R = U*V';
        T = [R [0 ; 0 ; 0] ; [xi(4:6)' 1]];
        tmp_tform = affine3d(T);
        
        %==== Updates the transformation and the pointcloud for the next iteration ====
        %==== (uses affine3d() and pctransform() functions) ====
        %==== (note the format of tform[] and the affine3d() function) ====
        tform = affine3d(tmp_tform.T*tform.T);
        if iter ~= iter_num
            new_pointcloud = pctransform(new_pointcloud, tmp_tform);
        end
        
    end
    
    %==== Find RMS error of point-plane registration ====
    error = sqrt(sum((A*xi - b).^2)/valid_pair_num);
end
        
