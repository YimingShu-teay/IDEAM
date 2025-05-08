import numpy as np
import copy
import time
import openpyxl

class decision:
    def __init__(self,l_diag,l,T_risk,epsilon,k,rho,a_max_acc_lon,a_max_brake_lon,a_min_brake_lon,threshold,d0,Td):
        self.graph = {
        'R1': ['C1', 'C2'],
        'R2': ['C1', 'C2'],
        'C1': ['L1', 'L2', 'R1', 'R2'],
        'C2': ['L1', 'L2', 'R1', 'R2'],
        'L1': ['C1', 'C2'],
        'L2': ['C1', 'C2']}
        
        self.group_list = ['L1', 'L2','C1', 'C2','R1', 'R2']
        
        self.l_diag = l_diag
        self.l = l
        self.T_risk = T_risk
        self.epsilon = epsilon
        self.k = k
        self.rho = rho
        self.a_max_acc_lon = a_max_acc_lon
        self.a_max_brake_lon = a_max_brake_lon
        self.a_min_brake_lon = a_min_brake_lon
        self.group_num = 6
        self.threshold = threshold
        self.d0 = d0
        self.Td = Td
    
    def remove_node_and_links(self, graph,node_to_remove):
        # 如果节点在图中，先移除这个节点
        if node_to_remove in graph:
            del graph[node_to_remove]

        # 遍历图中的所有节点，移除指向被删除节点的链接
        for node, edges in graph.items():
            if node_to_remove in edges:
                graph[node].remove(node_to_remove)
        return graph
        
    def find_all_paths(self, group_dict, graph, start, end, path=None, visited_higher=None, excluded=None, depth=1):

        if path is None:
            path = []
        if visited_higher is None:
            visited_higher = {}

        path = path + [start]
        visited_higher[start[0]] = max(visited_higher.get(start[0], 0), int(start[1]))

        if start == end:
            print("path=",path)
            return [path]

        paths = []
     
        for node in graph[start]:
            # print("start=",start)
            # print("node=",node)
            if node[0] in visited_higher and int(node[1]) <= visited_higher[node[0]]:
                continue             
            if node in excluded or not self.risk_assessment(group_dict, start, node, depth):
                # print("excluded=",excluded)
                # print("self.risk_assessment=",self.risk_assessment(group_dict, start, node, depth))
                continue

            newpaths = self.find_all_paths(group_dict, graph, node, end, path, visited_higher.copy(), excluded, depth + 1)
            for newpath in newpaths:
                print("newpath=",newpath)
                paths.append(newpath)
        print("paths=",paths)
        return paths
   

    def risk_assessment(self, group_dict, start, node, depth):
        # final_risk_judge： ture通过，false没通过
        # l：leader; f：follower; e:ego_gap; t: target_gap
        #要输出的是一个ture或者false
        start_group = group_dict[start]
        target_group = group_dict[node]
        
        group_vft = target_group['vf']
        group_vls = start_group['vl']
        
        group_sls = start_group['sl']
        group_sft = target_group['sf']
    

        if group_vft is None:
            final_risk_judge = True
            return final_risk_judge
        else:
            vls = group_vls
            vft = group_vft
            sls = group_sls  #sle有可能是None

            if sls is not None:
                d = group_sls - group_sft
                judge = []

                for i in range((depth-1)*self.T_risk,depth*self.T_risk):
                    if vls[i] >= vft[i]:
                        d_threshold = 2*self.l_diag + self.epsilon 
                        if d[i] >= d_threshold:     
                            judge.append(True)
                        else:
                            judge.append(False)
                    elif vft[i] > vls[i]:
                        dv = vft[i] - vls[i]
                        d_threshold = 2*self.l_diag + self.epsilon + self.k*dv
                        if d[i] >= d_threshold:                     
                            judge.append(True)
                        else:
                            judge.append(False)
                final_risk_judge = all(judge)
            else:
                final_risk_judge = True

            return final_risk_judge
        
    def gap_mag_judge(self,group_dict,start_group_str):
        #要输出的是gap列表
        exclusion_list = []
        for key, value in group_dict.items():
            final_mag_judge = self.gap_magnitude(value)
            if final_mag_judge == False:
                if key != start_group_str:
                    exclusion_list.append(key)
        return exclusion_list
        
    def gap_magnitude(self,group):
        
        group_vlt = group['vl']
        group_vft = group['vf']
        group_slt = group['sl']
        group_sft = group['sf']
        
        if group_vlt is None or group_vft is None:
            final_mag_judge = True
        else:
            vlt = group_vlt[0]
            dlg_s = self.Td*vlt + self.d0 + self.l
            dfg_s = self.d0 + self.l
            
            slt = group_slt[0]
            sft = group_sft[0]
            
            d_min_s = dlg_s + dfg_s - self.l
            d = slt - sft - self.l
            
            if d >= d_min_s:
                final_mag_judge = True
            else:
                final_mag_judge = False
        return final_mag_judge
    
    def long_term_efficiency(self,group_rest,group_list):
        long_term_s = []
        for key, value in group_rest.items():
            if value['sl'] is not None:
                s_last = value['sl'][-1]
            else:
                s_last = 10000
            long_term_s.append(s_last)
            
        index = np.argmax(np.array(long_term_s))
        long_term_result = group_list[index]
    # # 检查对应的 '2' 结尾的条目是否存在
    #     if long_term_result.endswith('1'):
    #         potential_result_2 = long_term_result[:-1] + '2'
    #         if potential_result_2 not in group_rest.keys():
    #             # 如果没有 '2' 结尾的条目，保持原有 '1' 结尾的结果
    #             return long_term_result
    #         else:
    #             # 如果存在 '2' 结尾的条目，则更新结果
    #             long_term_result = potential_result_2
        
        return long_term_result
    
    def short_term_efficiency(self, group_dict, shortest_paths, group_list):

        # 定义函数计算给定sl index时的最大值组名
        def compute_for_sl_index(sl_index):
            short_term_s = []
            short_term_group = []
            for item in shortest_paths:
                group_str = item[1]
                group = group_dict[group_str]
                if group['sl'] is not None and len(group['sl']) > sl_index:
                    s_last_short = group['sl'][sl_index]
                else:
                    s_last_short = 10000  # 用于处理没有足够数据的情况
                short_term_s.append(s_last_short)
                short_term_group.append(group)

            indices_sorted = np.argsort(-np.array(short_term_s))
            first_largest_index = indices_sorted[0]
            second_largest_index = indices_sorted[1] if len(indices_sorted) > 1 else first_largest_index
            
            return first_largest_index, second_largest_index, short_term_s, short_term_group

        # 按照不同的sl指数依次计算和比较
        for sl_index in [20, 30, 40, 50, 60]:
            first_largest_index, second_largest_index, short_term_s, short_term_group = compute_for_sl_index(sl_index)
            
            if len(short_term_s) > 1 and short_term_s[first_largest_index] - short_term_s[second_largest_index] < self.threshold:
                continue  # 如果差值小于阈值，则继续检查下一个sl指数
            else:
                # 找到满足条件的结果或在最后一个sl指数仍未满足条件时返回结果
                return short_term_group[first_largest_index]['name']
  
        # 如果所有检查都不满足，返回最终计算的最大值
        return short_term_group[first_largest_index]['name']
        
    def decision_making(self,group_dict,start_group_str):
        start_time = time.time()
        exclusion_list = self.gap_mag_judge(group_dict,start_group_str)
        group_rest = copy.deepcopy(group_dict)
        graph_copy = copy.deepcopy(self.graph)
        group_list_rest = copy.deepcopy(self.group_list)
 
        
        for key in self.group_list:
            if key in exclusion_list:
                del group_rest[key]
                graph_copy = self.remove_node_and_links(graph_copy,key)
                group_list_rest.remove(key)
     
        while True:
            long_term_result = self.long_term_efficiency(group_rest,group_list_rest)  # 计算长期效率结果
            # print("最好的结果:long_term_result=",long_term_result)
            
            # start_time = time.time()
            paths = self.find_all_paths(group_rest, graph_copy, start_group_str, long_term_result, excluded=exclusion_list)  # 尝试找到路径
            # end_time = time.time()
            # DFS_duration = end_time - start_time

            # DFS_file = "dfs_times.xlsx"
            # # 记录 decision making 时间到 Excel 文件
            # wb = openpyxl.load_workbook(DFS_file)
            # sheet = wb["DFS Times"]
            # sheet.append(["DFS Making", DFS_duration])
            # wb.save(DFS_file)
            
            if long_term_result == start_group_str:
                end_time = time.time()
                decision_making_duration = end_time - start_time

                decision_file = "decision_times.xlsx"
                # 记录 decision making 时间到 Excel 文件
                wb = openpyxl.load_workbook(decision_file)
                sheet = wb["Decision Times"]
                sheet.append(["Decision Making", decision_making_duration])
                wb.save(decision_file)
                return group_dict[start_group_str]
            
            if paths != []:  
                min_length = min(len(path) for path in paths)

                min_indices = [i for i, path in enumerate(paths) if len(path) == min_length]
                shortest_paths = [paths[i] for i in min_indices]

                if len(shortest_paths) == 1:
                    short_term_result = shortest_paths[0][1]
                    desired_group = group_dict[short_term_result]
                else:
                    short_term_result = self.short_term_efficiency(group_rest,shortest_paths,group_list_rest)
                    
                    desired_group = group_dict[short_term_result]
                    
                end_time = time.time()
                decision_making_duration = end_time - start_time

                decision_file = "decision_times.xlsx"
                # 记录 decision making 时间到 Excel 文件
                wb = openpyxl.load_workbook(decision_file)
                sheet = wb["Decision Times"]
                sheet.append(["Decision Making", decision_making_duration])
                wb.save(decision_file)
                return desired_group

            else:
                del group_rest[long_term_result]
                graph_copy = self.remove_node_and_links(graph_copy,long_term_result)
                group_list_rest.remove(long_term_result)
            
        
    
