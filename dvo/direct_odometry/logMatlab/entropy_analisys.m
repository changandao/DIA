clear all;
close all;
%%
figure
[entropy,e_t] = textread('entropyLog.txt','%f %f'); 
plot(entropy,e_t,'.r');
%%
% entropy_intervals = min(entropy):0.5:max(entropy);
% 
% for i = 1:1:length(entropy_intervals)-1
%     error_mean = 0;
%     errors_i = [];
%     for j = 1:1:length(entropy(:,1))
%         if (entropy(j)>entropy_intervals(i))&&(entropy(j)<entropy_intervals(i+1))
%             errors_i = [errors_i, e_t(j)];
%         end
%     end
%    
%     mean_error(i) = mean(errors_i);
%     std_error(i) = std(errors_i);
% 
% end
% figure
% plot(entropy_intervals(1:end-1),mean_error,'.b');
% hold on;
% plot(entropy_intervals(1:end-1),mean_error+std_error,'r');
% plot(entropy_intervals(1:end-1),mean_error-std_error,'r');
%%
entropy_intervals = [];
entropy_intervals = min(entropy):0.1:max(entropy);
for i = 1:1:length(entropy_intervals)-1
    entropy_i = entropy_intervals(i);
    errors_i = [];
    for j = 1:1:length(entropy(:,1))
        if (entropy(j)>entropy_intervals(i))
            errors_i = [errors_i, e_t(j)];
        end
    end
    mean_error(i) = mean(errors_i);
    std_error(i) = std(errors_i);
end
figure
plot(entropy_intervals(2:end),mean_error,'.b');
hold on;
plot(entropy_intervals(2:end),mean_error+std_error,'r');

