from pandas import read_csv
import matplotlib.pyplot as plt
import numpy as np
import scipy.stats as st

def readFile(filename):
    data = read_csv(filename, header=None)
    float_data = data.values.astype(float)
    return float_data
# Read CSV file
data = readFile('force_test.csv')

test1 = data[0]
test2 = data[1]
test3 = data[2]
test4 = data[3]
test5 = data[4]
test6 = data[5]
test7 = data[6]
test8 = data[7]
test9 = data[8]

# Remove nAn values
test1 = test1[~np.isnan(test1)]
test2 = test2[~np.isnan(test2)]
test3 = test3[~np.isnan(test3)]
test4 = test4[~np.isnan(test4)]
test5 = test5[~np.isnan(test5)]
test6 = test6[~np.isnan(test6)]
test7 = test7[~np.isnan(test7)]
test8 = test8[~np.isnan(test8)]
test9 = test9[~np.isnan(test9)]

# Calculate mean
mean1 = np.mean(test1)
mean2 = np.mean(test2)
mean3 = np.mean(test3)
mean4 = np.mean(test4)
mean5 = np.mean(test5)
mean6 = np.mean(test6)
mean7 = np.mean(test7)
mean8 = np.mean(test8)
mean9 = np.mean(test9)

# Calculate standard deviation
std1 = np.std(test1)
std2 = np.std(test2)
std3 = np.std(test3)
std4 = np.std(test4)
std5 = np.std(test5)
std6 = np.std(test6)
std7 = np.std(test7)
std8 = np.std(test8)
std9 = np.std(test9)

# Calculate standard error
se1 = std1 / np.sqrt(len(test1))
se2 = std2 / np.sqrt(len(test2))
se3 = std3 / np.sqrt(len(test3))
se4 = std4 / np.sqrt(len(test4))
se5 = std5 / np.sqrt(len(test5))
se6 = std6 / np.sqrt(len(test6))
se7 = std7 / np.sqrt(len(test7))
se8 = std8 / np.sqrt(len(test8))
se9 = std9 / np.sqrt(len(test9))

# Calculate total mean, standard deviation and standard error
all_values = np.concatenate((test1, test2, test3, test4, test5, test6, test7, test8, test9), axis=None)
total_mean = np.mean(all_values)
total_std = np.std(all_values)
total_se = total_std / np.sqrt(len(all_values))
# total_mean = np.mean([mean1, mean2, mean3, mean4, mean5, mean6, mean7, mean8, mean9])
# total_std = np.std([std1, std2, std3, std4, std5, std6, std7, std8, std9])
# total_se = np.std([se1, se2, se3, se4, se5, se6, se7, se8, se9])

# Confidence interval of 95%
ci1 = st.t.interval(confidence=0.95, df=len(test1)-1, loc=mean1, scale=se1)
ci2 = st.t.interval(confidence=0.95, df=len(test2)-1, loc=mean2, scale=se2)
ci3 = st.t.interval(confidence=0.95, df=len(test3)-1, loc=mean3, scale=se3)
ci4 = st.t.interval(confidence=0.95, df=len(test4)-1, loc=mean4, scale=se4)
ci5 = st.t.interval(confidence=0.95, df=len(test5)-1, loc=mean5, scale=se5)
ci6 = st.t.interval(confidence=0.95, df=len(test6)-1, loc=mean6, scale=se6)
ci7 = st.t.interval(confidence=0.95, df=len(test7)-1, loc=mean7, scale=se7)
ci8 = st.t.interval(confidence=0.95, df=len(test8)-1, loc=mean8, scale=se8)
ci9 = st.t.interval(confidence=0.95, df=len(test9)-1, loc=mean9, scale=se9)
ci_total = st.t.interval(confidence=0.95, df=len(all_values)-1, loc=total_mean, scale=total_se)

# Print results
print('Test 1: mean = %.2f, standard deviation = %.2f, standard error = %.2f, confidence interval = %.2f - %.2f' % (mean1, std1, se1, ci1[0], ci1[1]))
print('Test 2: mean = %.2f, standard deviation = %.2f, standard error = %.2f, confidence interval = %.2f - %.2f' % (mean2, std2, se2, ci2[0], ci2[1]))
print('Test 3: mean = %.2f, standard deviation = %.2f, standard error = %.2f, confidence interval = %.2f - %.2f' % (mean3, std3, se3, ci3[0], ci3[1]))
print('Test 4: mean = %.2f, standard deviation = %.2f, standard error = %.2f, confidence interval = %.2f - %.2f' % (mean4, std4, se4, ci4[0], ci4[1]))
print('Test 5: mean = %.2f, standard deviation = %.2f, standard error = %.2f, confidence interval = %.2f - %.2f' % (mean5, std5, se5, ci5[0], ci5[1]))
print('Test 6: mean = %.2f, standard deviation = %.2f, standard error = %.2f, confidence interval = %.2f - %.2f' % (mean6, std6, se6, ci6[0], ci6[1]))
print('Test 7: mean = %.2f, standard deviation = %.2f, standard error = %.2f, confidence interval = %.2f - %.2f' % (mean7, std7, se7, ci7[0], ci7[1]))
print('Test 8: mean = %.2f, standard deviation = %.2f, standard error = %.2f, confidence interval = %.2f - %.2f' % (mean8, std8, se8, ci8[0], ci8[1]))
print('Test 9: mean = %.2f, standard deviation = %.2f, standard error = %.2f, confidence interval = %.2f - %.2f' % (mean9, std9, se9, ci9[0], ci9[1]))
print('Total: mean = %.2f, standard deviation = %.2f, standard error = %.2f, confidence interval = %.2f - %.2f' % (total_mean, total_std, total_se, ci_total[0], ci_total[1]))
# Make a list of means, standard deviations and standard errors
means = [mean1, mean2, mean3, mean4, mean5, mean6, mean7, mean8, mean9, total_mean]
stds = [std1, std2, std3, std4, std5, std6, std7, std8, std9, total_std]
ses = [se1, se2, se3, se4, se5, se6, se7, se8, se9, total_se]

# Make a list of labels
labels = ['1', '2', '3', '4', '5', '6', '7', '8', '9', 'Total']

# Plot the means with standard errors
plt.figure(1)
plt.bar(labels, means, yerr=ses, align='center', alpha=0.5, ecolor='black', capsize=10)
plt.ylabel('Force (kgf)')
plt.xlabel('Test')
plt.title('Mean and standard error of force test')
plt.show()

# Plot the standard deviations
plt.figure(2)
plt.bar(labels, stds, align='center', alpha=0.5, ecolor='black', capsize=10)
plt.ylabel('Force (kgf)')
plt.xlabel('Test')
plt.title('Standard deviation of force test')
plt.show()

# Plot the standard errors
plt.figure(3)
plt.bar(labels, ses, align='center', alpha=0.5, ecolor='black', capsize=10)
plt.ylabel('Force (kgf)')
plt.xlabel('Test')
plt.title('Standard error of force test')
plt.show()

# Plot the confidence intervals
plt.figure(4)







