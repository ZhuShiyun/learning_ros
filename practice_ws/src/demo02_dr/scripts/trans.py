import json

# 你的示例数据
data = [
    "3366190.099279,544915.235681,-1.895000,-0.038242",
    "3366190.099279,544915.235681,-1.895000,-0.038242",
    "-0.046250,-0.108000",
    "0.800000,0.131125"
]

# 初始化一个空的字典用于存储转换后的数据
result = {}

# 解析示例数据
for i, line in enumerate(data):
    params = line.split(',')
    for j, param in enumerate(params):
        key = f"param{j}"
        if key not in result:
            result[key] = []
        result[key].append(param)

# 添加 "x" 和 "y" 键
result["x"] = result["param0"][0]
result["y"] = result["param1"][0]

# 移除不需要的键
result.pop("param0", None)
result.pop("param1", None)

# 转换为 JSON 格式
json_data = json.dumps(result, indent=4)

# 打印转换后的 JSON 数据
# print(json_data)

# 将数据保存到 JSON 文件
with open('output.json', 'w') as json_file:
    json.dump(result, json_file, indent=4)

