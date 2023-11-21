import pandas as pd
import os


def find_all_files(directory):
    directory = f"storage/logs/{directory}"
    all_files = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            if not file.endswith('csv'):
                all_files.append(os.path.join(root, file))
    return all_files


def column_mean(df, column_index):
    total = 0.0
    i = 0
    for val in df.iloc[:, column_index]:
        total += float(val)
        i += 1
    return total / i


def column_mean_conditional(df, column_index, condition_column, condition_value):
    total = 0.0
    i = 0
    for ind, val in enumerate(df.iloc[:, column_index]):
        check = float(df.iloc[ind, condition_column])
        if check == condition_value:
            total += float(val)
            i += 1
    if i == 0:
        return 0
    return total / i


def column_std(df):
    return df['Time'].std()


def parse_log(file_name):
    if "storage/logs" not in file_name:
        path = f"storage/logs/{file_name}"
    else:
        path = file_name
    file = open(path, "r+")

    data_ready = False
    header_ready = False
    header_list = []
    data_list = []
    for line in file.readlines():
        if line.endswith(" properties for each run\n"):
            header_ready = True
            continue
        elif line.endswith(" runs\n"):
            header_ready = False
            data_ready = True
            continue

        if header_ready:
            header_list.append(line.split(" ")[0].replace('_', ' ').title())
        elif data_ready:
            if line == "." or line == ".\n":
                break
            data = line.replace(";\n", "").split("; ")
            temp_data = []
            for r in data:
                if r != '\n':
                    try:
                        temp_data.append(float(r))
                    except:
                        temp_data.append(r)
            data_list.append(temp_data)
    return pd.DataFrame(data_list[1:], columns=header_list)


directory_name = input("Enter directory name:\t")

all_files = find_all_files(directory_name)


for file_name in all_files:
    df = parse_log(file_name)

    df.to_csv(f"{file_name}.csv", index=False)

    # PRINT

    print(f"\n===  RESULTS   for {file_name.split('/')[-1]}")
    mean = column_mean(df, 0)
    print(f" Average Time of All Results: \t\t\t  {mean:.2f}")

    time_mean_conditional = column_mean_conditional(df, 0, 1, 1)
    print(f" Average Time of Successful Results  \t  {time_mean_conditional:.2f}")

    success_rate = df['Success'].sum()
    number_of_rows = len(df)
    print(f" Number Success / All Results \t\t\t  {int(success_rate)}/{number_of_rows}")

    std_suc = df[df['Success'] == 1]['Time'].std()
    print(f" Std.Dev. of Time of Successful Results \t  {std_suc:.2f}")

    std = column_std(df)
    print(f" Standard Deviation of Time \t\t\t  {std:.2f}")

    length_mean_conditional = column_mean_conditional(df, 2, 1, 1)
    print(f" Average Length of Successful Results \t  {length_mean_conditional:.2f}")
    print()




