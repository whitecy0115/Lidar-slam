% function scan = getScanData(pRPLIDAR)
% count = 0;
% alldistances = zeros([1,362]);
% allangles = zeros([1,362]);
% 
% tic;
% i=1;
% while 1
%     [~, distances, angles, ~, ~] = GetScanDataResponseRPLIDAR(pRPLIDAR);  
%     alldistances(i) = distances;
%     allangles(i) =angles; 
%    
%     if count > 360*2
%     %if count > 720/32
%        idx = find(-0.523 <= allangles & 0.523 >= allangles);
%        allangles =  allangles(1,idx);
%        alldistances = alldistances(1,idx);
%        scan = lidarScan(alldistances, allangles);
%        break;
%     end    
%     count = count+1;
%     i=i+1;
% end
% end


function flag = getScanData(pRPLIDAR)

 flag=0;
 count=0;
      
   for i=1:360

       [~, distances, angles, ~, ~] = GetScanDataResponseRPLIDAR(pRPLIDAR);  
        
        if -0.2618 <= angles && 0.2618 >= angles && 0.4 <= distances && 0.7 >= distances 
            count = count +1;
        end

        if count > 35
            flag =1;
            break;

        end
   end

end