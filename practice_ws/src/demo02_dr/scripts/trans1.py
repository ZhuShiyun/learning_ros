# import json

# # 读取.log文件内容
# with open('/home/b-zhushiyun/myWorkDir/817log/test.log', 'r') as log_file:
#     lines = log_file.readlines()

# # 初始化一个空的json列表，用于存储最终的JSON数据
# json_data = []

# # 遍历.log文件的内容，每四行处理一组数据
# for i in range(0, len(lines), 4):
#     x, y, _, param1, param2, param3, param4, param5 = map(float, lines[i+1].strip().split(','))
    
#     # 构建JSON对象
#     data = {
#         "x": str(x),
#         "y": str(y),
#         "param0": str(lines[i].strip()),
#         "param1": str(param1),
#         "param2": str(param2),
#         "param3": str(param3),
#         "param4": str(param4),
#         "param5": str(param5)
#     }
    
#     # 将JSON对象添加到json_data列表中
#     json_data.append(data)

# # 将json_data写入.json文件
# with open('output.json', 'w') as json_file:
#     json.dump(json_data, json_file, indent=4)

import json

# 读取.log文件内容
with open('/home/b-zhushiyun/myWorkDir/817log/testlog2.log', 'r', encoding='utf-8') as log_file:
    lines = log_file.readlines()

# 初始化一个空的json列表，用于存储最终的JSON数据
json_data = []

# 遍历.log文件的内容，每四行处理一组数据
for i in range(0, len(lines), 4):
    temp=""
    for j in range(0,4):
        temp += lines[i+j]
    #print(temp.strip().replace('\n',',').split(','))
    _, _, _, _, x, y, param0, param1, param2, param3, param4, param5 = temp.strip().replace('\n',',').split(',')
    
    # 构建JSON对象
    data = {
    "x": str(x),
    "y": str(y),
    "param0": str(param0),
    "param1": str(param1),
    "param2": str(param2),
    "param3": str(param3),
    "param4": str(param4),
    "param5": str(param5)
    }

        # 将JSON对象添加到json_data列表中
    json_data.append(data)

# 将json_data写入.json文件
with open('output_tran3.json', 'w') as json_file:
    json.dump(json_data, json_file, indent=4)
