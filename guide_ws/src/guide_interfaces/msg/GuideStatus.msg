string state          # idle | running | paused | stopping | finished | error
string stage          # navigating | heading | speaking | dwelling | returning
string poi_name
int32 index           # 当前点位序号（从 1 开始，0 表示不在点位列表）
int32 total
string message
