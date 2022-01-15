function [error]=error_calc(results,target,line,j,max_rotation)
            same_line=false;
            spin=results(end,7:9);
            hit=[results(end,1:3),results(end,11:14)];
            target_pos=target(1:3);
            target_quat=target(4:7);
            pos=hit(1:3);
            quat=hit(4:7);
            normalised_pos=(pos./target_pos); %so proportion of total dist considered
            normalised_quat=quat./target_quat;
            normalised=[normalised_pos,normalised_quat];
            for k=1:numel(normalised) %removes NaN errors and inf values
                if isnan(normalised(k))
                    normalised(k)=1;
                end
                if isinf(normalised(k))
                    if hit(k)<1
                    normalised(k)=1-hit(k); %if less than 1 will have small effect
                    else %if pos aiming for 0 
                        normalised(k)=1-hit(k)/norm(hit-target);
                    end
                end
            end
            pos_check=abs((1-normalised(1:3)).*((pos-target_pos)./norm(pos-target_pos)));
            quat_check=abs(1-normalised(4:7));
            spin_mag=norm(spin);
            spin_check=spin_mag/max_rotation;
                %% check if on same line
                %find grad of lines
        final_line=results(end,1:3)-results(end-1,1:3);
        path_line=line(j+1,:)-line(j,:);
        final_line_d=round(final_line/norm(final_line),3); %line grad to 3dp
        path_line_d=round(path_line/norm(path_line),3);
        
        if path_line_d==final_line_d %is parrallel or same 
            %check if test final satisfies path line
            d=(round(results(end,3),3)-line(j+1,3))/path_line(3); %find distance along line
            test1=(round(results(end,1),3)==path_line(1)+d*path_line(1)); %check if point on line with distance
            test2=(round(results(end,2),3)==path_line(2)+d*path_line(2));
            if and(test1,test2)    
                same_line=true;    %position error fine as in right direction - only check quat
            end
        end
        %% calc error
        if same_line
            error=norm(quat_check); %spin magnitude to ensure stable flight
        else
                error=norm([pos_check,quat_check,(spin_check/2)]);
        end
end