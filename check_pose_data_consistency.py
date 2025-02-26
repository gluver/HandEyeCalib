import pandas as pd
import sys
#given two file paths , read the data from the files based on their extension(csv,xlsx,txt), compare if all data is consistent and return True if it is consistent else False
def check_pose_data_consistency(file_path1, file_path2):
    if file_path1.endswith('.csv'):
        data1 = pd.read_csv(file_path1, header=None)
    elif file_path1.endswith('.xlsx'):
        data1 = pd.read_excel(file_path1)
    elif file_path1.endswith('.txt'):
        data1 = pd.read_csv(file_path1, sep='\t')
    else:
        return False

    if file_path2.endswith('.csv'):
        data2 = pd.read_csv(file_path2)
    elif file_path2.endswith('.xlsx'):
        data2 = pd.read_excel(file_path2)
    elif file_path2.endswith('.txt'):
        data2 = pd.read_csv(file_path2, sep='\t')
    else:
        return False
    # print(data1.values,'\n',data2.values)
    if data1.shape != data2.shape:
        return False

    for col in range(data1.shape[1]):
        for row in range(data1.shape[0]):
            if data1.iat[row, col] != data2.iat[row, col]:
                return False

    return True

if __name__ == '__main__':
    # read command from the user
    if len(sys.argv) != 3:
        print("Usage: python check_pose_data_consistency.py <file_path1> <file_path2>")
        sys.exit(1)
    file_path1 = sys.argv[1]
    file_path2 = sys.argv[2]
    # check if the data is consistent
    if check_pose_data_consistency(file_path1, file_path2):
        print('The data is consistent')
    else:
        print('The data is not consistent')