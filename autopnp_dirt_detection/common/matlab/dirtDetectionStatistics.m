% for each surface material
%   for each dirt threshold
%     get a recall-precision curve over all kinds of dirt
%     select best dirt threshold for this floor material
% compute final dirt recognition recall-precision curve
function dirtDetectionStatistics()

    % parameters
    % ---------------------------------------------------------------------------------------------------------------------------------
    path = '../files/results/with_warp/';   % path to results folder
    floorMaterials = {'carpet';'corridor';'kitchen';'linoleum';'office'}
    dirtTypes = {'clean';'food';'fuzz';'leaves';'office';'paper1';'paper2';'paper3';'stones';'streetdirt'}
    dirtThresholdMin = 0.1;     % these numbers should correspond with those in the C++ code
    dirtThresholdMax = 0.5;
    dirtThresholdStep = 0.05;
    neighborhoodSize = 2;   % radius of neighborhood in which ground truth dirt has to be found for a detection to count as correct
    findBestThresholds = 0; % if this is set to 1, the algorithm searches for the best dirt threshold for each floor type
    fixedThreshold = 0.15;  % if findBestThresholds==0, this fixed dirt threshold is used for all data
    % ---------------------------------------------------------------------------------------------------------------------------------

    % storage
    dirtThresholds = dirtThresholdMin:dirtThresholdStep:dirtThresholdMax;
    gt = cell(size(floorMaterials, 1),size(dirtTypes,1));   % ground truth map (0=no dirt, >0=there is some dirt)
    bestDirtThreshold = [];
    if (findBestThresholds ~= 0)
       bestDirtThreshold = [zeros(size(floorMaterials, 1), 1), 2.0*ones(size(floorMaterials, 1), 1)];    % remembers the best dirtThreshold for each floor material (1st column = best threshold, 2nd column = distance to ideal point)
    else
       bestDirtThreshold = [fixedThreshold*ones(size(floorMaterials, 1), 1), 2.0*ones(size(floorMaterials, 1), 1)];    % remembers the best dirtThreshold for each floor material (1st column = best threshold, 2nd column = distance to ideal point)
    end
    
    % load maps and find best dirtThreshold for each floor material
    if (findBestThresholds==1)
        for mat=1:size(floorMaterials,1)
            disp(floorMaterials{mat})
            % ground truth
            for dirt=1:size(dirtTypes,1)
                gt(mat,dirt) = {load([path, floorMaterials{mat}, '-', dirtTypes{dirt}, '-gt.map'])};
            end
            % find best dirt threshold
            for thresholdIndex=1:size(dirtThresholds,2)
                disp(['  threshold: ', num2str(dirtThresholds(thresholdIndex))])
                tp_=zeros(size(dirtTypes,1),100);
                fn_=zeros(size(dirtTypes,1),100);
                fp_=zeros(size(dirtTypes,1),100);
                tn_=zeros(size(dirtTypes,1),100);
                for dirt=1:size(dirtTypes,1)
                    disp(['    ', dirtTypes{dirt}])
                    % detections
                    pv = load([path, floorMaterials{mat}, '-', dirtTypes{dirt}, '-dt', num2str(dirtThresholds(thresholdIndex)), '-pv.map']);  % number of positive votes per cell
                    no = load([path, floorMaterials{mat}, '-', dirtTypes{dirt}, '-dt', num2str(dirtThresholds(thresholdIndex)), '-no.map']);  % number of observations per cell
                    detections = (100.0*pv)./no;

                    % compute a recall-precision curve and find closest curve point to ideal point
                    recall = [];
                    precision = [];
                    for detectionThreshold=1:1:100
                        % compute recall-precision
                        [tp_(dirt,detectionThreshold),fn_(dirt,detectionThreshold),fp_(dirt,detectionThreshold),tn_(dirt,detectionThreshold),recall_,precision_] = computeStatistics(gt{mat,dirt}, detections, detectionThreshold, neighborhoodSize);
                        recall = [recall, recall_];
                        precision = [precision, precision_];
                    end
    %                 close all
    %                 figure
    %                 plot(precision, recall, '-o');
    %                 grid
    %                 xlim([0,1])
    %                 ylim([0,1])
                end
    %             close all
    %             figure
    %             imshow(gt{mat,dirt});
    %             figure
    %             imshow(pv,[min(min(pv)), max(max(pv))]);
                for detectionThreshold=1:1:100
                    tp=sum(tp_(:,detectionThreshold),1);
                    fn=sum(fn_(:,detectionThreshold),1);
                    fp=sum(fp_(:,detectionThreshold),1);
                    tn=sum(tn_(:,detectionThreshold),1);
                    if (tp+fn==0)
                        recall=1;
                    else
                        recall = tp/(tp+fn);
                    end
                    if (tp==0 || tp+fp==0)
                        precision=1;
                    else
                        precision = tp/(tp+fp);
                    end
                    dist = norm([1,1]-[precision,recall]);
                    if (dist < bestDirtThreshold(mat,2))
                        bestDirtThreshold(mat,2) = dist;
                        bestDirtThreshold(mat,1) = dirtThresholds(thresholdIndex);
                    end
                end
            end
        end

        disp('Best dirt thresholds:')
        bestDirtThreshold
    end
    
    % compute recall-precision curve for best dirtThresholds
    tpMaterial=zeros(size(floorMaterials,1), 100);
    fnMaterial=zeros(size(floorMaterials,1), 100);
    fpMaterial=zeros(size(floorMaterials,1), 100);
    tnMaterial=zeros(size(floorMaterials,1), 100);
    tpDirt=zeros(size(dirtTypes,1),100);
    fnDirt=zeros(size(dirtTypes,1),100);
    fpDirt=zeros(size(dirtTypes,1),100);
    tnDirt=zeros(size(dirtTypes,1),100);
    for mat=1:size(floorMaterials,1)
        % ground truth
        for dirt=1:size(dirtTypes,1)
            gt(mat,dirt) = {load([path, floorMaterials{mat}, '-', dirtTypes{dirt}, '-gt.map'])};
        end
        
        % use best dirt threshold for each class
        dirtThreshold = bestDirtThreshold(mat,1);
        tp_=zeros(size(dirtTypes,1),100);
        fn_=zeros(size(dirtTypes,1),100);
        fp_=zeros(size(dirtTypes,1),100);
        tn_=zeros(size(dirtTypes,1),100);
        for dirt=1:size(dirtTypes,1)
            % detections
            pv = load([path, floorMaterials{mat}, '-', dirtTypes{dirt}, '-dt', num2str(dirtThreshold), '-pv.map']);  % number of positive votes per cell
            no = load([path, floorMaterials{mat}, '-', dirtTypes{dirt}, '-dt', num2str(dirtThreshold), '-no.map']);  % number of observations per cell
            detections = (100.0*pv)./no;
            % compute a recall-precision curve
            for detectionThreshold=1:1:100
                [tp_(dirt,detectionThreshold),fn_(dirt,detectionThreshold),fp_(dirt,detectionThreshold),tn_(dirt,detectionThreshold),recall_,precision_] = computeStatistics(gt{mat,dirt}, detections, detectionThreshold, neighborhoodSize);
            end
        end
%             close all
%             figure
%             imshow(gt{mat,dirt});
%             figure
%             imshow(pv,[min(min(pv)), max(max(pv))]);
        
        tpMaterial(mat,:) = sum(tp_,1);
        fnMaterial(mat,:) = sum(fn_,1);
        fpMaterial(mat,:) = sum(fp_,1);
        tnMaterial(mat,:) = sum(tn_,1);
        tpDirt = tpDirt + tp_;
        fnDirt = fnDirt + fn_;
        fpDirt = fpDirt + fp_;
        tnDirt = tnDirt + tn_;
    end        

    tpAll = sum(tpMaterial,1);
    fnAll = sum(fnMaterial,1);
    fpAll = sum(fpMaterial,1);
    tnAll = sum(tnMaterial,1);

    % total
    [recallTotal, precisionTotal] = computeRecallPrecision(tpAll, fnAll, fpAll);
%     figure 
%     plot(precisionTotal, recallTotal, '.-', 'LineWidth', 1.5);
%     grid
%     xlim([0,1])
%     ylim([0,1])
%     xlabel('Precision')
%     ylabel('Recall')
    
    % per material
    recallMaterial = [];
    precisionMaterial = [];
    for mat=1:size(floorMaterials,1)
        [recall_, precision_] = computeRecallPrecision(tpMaterial(mat,:), fnMaterial(mat,:), fpMaterial(mat,:));
        recallMaterial = [recallMaterial; recall_];
        precisionMaterial = [precisionMaterial; precision_];
    end
    figure 
    plot([precisionTotal; precisionMaterial]', [recallTotal; recallMaterial]', '.-');
    xlim([0,1])
    ylim([0,1])
    grid
    xlabel('Precision')
    ylabel('Recall')
    legend(['average'; floorMaterials])

    % per dirt type
    recallDirt = [];
    precisionDirt = [];
    for dirt=2:size(dirtTypes,1)
        [recall_, precision_] = computeRecallPrecision(tpDirt(dirt,:), fnDirt(dirt,:), fpDirt(dirt,:));
        recallDirt = [recallDirt; recall_];
        precisionDirt = [precisionDirt; precision_];
    end
    figure 
    plot(precisionDirt', recallDirt', '.-');
    grid
    xlim([0,1])
    ylim([0,1])
    xlabel('Precision')
    ylabel('Recall')
    legend(dirtTypes{2:end})
end

function [tp,fn,fp,tn,recall,precision]=computeStatistics(gt, det, threshold, neighborhoodSize)
    tp=0;
    fn=0;
    fp=0;
    tn=0;
    
    for v=1:size(gt,1)
        for u=1:size(gt,2)
            val = det(v,u);
            if (isnan(val)==1)
                continue;
            end
            
            gtDirtClose=0;
            for dv=-neighborhoodSize:1:neighborhoodSize
                for du=-neighborhoodSize:1:neighborhoodSize
                    if (v+dv>0 && v+dv<=size(gt,1) && u+du>0 && u+du<=size(gt,2))
                        if (gt(v+dv,u+du)>0)
                            gtDirtClose=1;
                        end
                    end
                end
            end
            
            if (gt(v,u)==0 && val<threshold)
                tn = tn+1;
            elseif (gtDirtClose==1 && val>=threshold)
                tp = tp+1;
            elseif (gtDirtClose==0 && val>=threshold)
                fp = fp+1;
            else
                fn = fn+1;
            end
        end
    end
    
    if (tp+fn==0)
        recall=1;
    else
        recall = tp/(tp+fn);
    end
    if (tp==0 || tp+fp==0)
        precision=1;
    else
        precision = tp/(tp+fp);
    end
end


function [recall, precision] = computeRecallPrecision(tp,fn,fp)
    recall = ones(size(tp));
    precision = ones(size(tp));
    for i=1:size(tp,2);
        if (tp(i)+fn(i)==0)
            recall(i)=1;
        else
            recall(i) = tp(i)/(tp(i)+fn(i));
        end
        if (tp(i)==0 || tp(i)+fp(i)==0)
            precision(i)=1;
        else
            precision(i) = tp(i)/(tp(i)+fp(i));
        end
    end
end