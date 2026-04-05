#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <algorithm>
#include <ctime>
#include <sstream>
#include <cstdlib>

class ObjectSelector
{
public:
    ObjectSelector(ros::NodeHandle& nh) : nh_(nh), category_(""), last_selected_("")
    {
        // 类别到物品映射
        category_to_objects_ = {
            {"fruit", {"apple", "banana", "watermelon"}},
            {"vegetable", {"pepper", "tomato", "potato"}},
            {"dessert", {"milk", "cake", "coke"}}
        };

        // 物品名纠正映射
        category_correction_ = {
            {"milkdata", "milk"},
            {"cokedata", "coke"},
            {"whatermelondata", "watermelon"},
            {"watermelondata", "watermelon"},
            {"pepperdata", "pepper"},
            {"tomatodata", "tomato"},
            {"potatodata", "potato"},
            {"appledata", "apple"},
            {"bananadata", "banana"},
            {"cakedata", "cake"}
        };

        // 初始化订阅者和发布者
        category_sub_ = nh_.subscribe("/task_status/category", 1, &ObjectSelector::categoryCb, this);
        yolo_sub_ = nh_.subscribe("/yolo_objects", 1, &ObjectSelector::yoloCb, this);
        result_pub_ = nh_.advertise<std_msgs::String>("/selected_object", 1);
        speech_done_pub_ = nh_.advertise<std_msgs::Bool>("/speech_done", 1);

        // 每次启动时清空日志文件
        clearLogFile();
        
        ROS_INFO("【object_selector】节点已启动，等待类别和识别数据...");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber category_sub_;
    ros::Subscriber yolo_sub_;
    ros::Publisher result_pub_;
    ros::Publisher speech_done_pub_;

    std::string category_;                   // 本次任务类别
    std::vector<std::string> detected_objs_; // 识别结果(不重复)
    std::string last_selected_;              // 上一次发布的物体

    std::map<std::string, std::vector<std::string>> category_to_objects_;
    std::map<std::string, std::string> category_correction_;

    std::string log_path_ = "/home/iflytek/log/object_detect_history.txt"; // 日志路径

    // 去除前后空格并转为小写
    std::string trim_lower(const std::string& s)
    {
        size_t start = s.find_first_not_of(" \t\r\n");
        size_t end = s.find_last_not_of(" \t\r\n");
        std::string t = (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
        std::transform(t.begin(), t.end(), t.begin(), ::tolower);
        return t;
    }

    // 清空日志文件
    void clearLogFile()
    {
        std::ofstream fout(log_path_, std::ios::trunc);
        if (fout)
        {
            ROS_INFO("【object_selector】日志文件已清空: %s", log_path_.c_str());
        }
        else
        {
            ROS_WARN("【object_selector】无法清空日志文件: %s", log_path_.c_str());
        }
    }

    void categoryCb(const std_msgs::String::ConstPtr& msg)
    {
        category_ = trim_lower(msg->data);
        ROS_INFO("【object_selector】收到二维码任务类别: %s", category_.c_str());
    }

    void yoloCb(const std_msgs::String::ConstPtr& msg)
    {
        if (category_.empty())
        {
            ROS_WARN("【object_selector】还未收到二维码任务类别，暂不处理识别结果");
            return;
        }

        // 如果已经收集到3个不重复物体，不再处理新数据
        if (detected_objs_.size() >= 3)
        {
            return;
        }

        // 拆分识别物体, 逗号分隔，名称标准化
        std::string str = msg->data;
        std::vector<std::string> objs;
        size_t start = 0, end;
        while ((end = str.find(',', start)) != std::string::npos)
        {
            std::string obj = trim_lower(str.substr(start, end - start));
            auto it = category_correction_.find(obj);
            if (it != category_correction_.end())
                objs.push_back(it->second);
            else
                objs.push_back(obj);
            start = end + 1;
        }
        // 处理最后一个物体
        if (start < str.size())
        {
            std::string obj = trim_lower(str.substr(start));
            auto it = category_correction_.find(obj);
            if (it != category_correction_.end())
                objs.push_back(it->second);
            else
                objs.push_back(obj);
        }

        // 添加不重复的物体
        for (const auto& obj : objs)
        {
            // 检查是否已存在
            if (std::find(detected_objs_.begin(), detected_objs_.end(), obj) == detected_objs_.end())
            {
                detected_objs_.push_back(obj);
                // 保持列表最多3个元素
                if (detected_objs_.size() > 3)
                {
                    detected_objs_.pop_back();
                }
            }
        }

        ROS_INFO("【object_selector】收到识别物体: [%s]（当前累计不重复共%lu个）",
                 join(objs, ", ").c_str(), detected_objs_.size());

        // 收集满3个不重复物体后进行处理
        if (detected_objs_.size() == 3)
        {
            ROS_INFO("【object_selector】收集完3个不重复物体: [%s]", join(detected_objs_, ", ").c_str());

            // 记录到本地日志文件（带时间戳）
            writeDetectLog(detected_objs_);

            // 判断属于本类别的物体
            std::string selected;
            auto it = category_to_objects_.find(category_);
            if (it != category_to_objects_.end())
            {
                for (const auto& obj : detected_objs_)
                {
                    if (std::find(it->second.begin(), it->second.end(), obj) != it->second.end())
                    {
                        selected = obj;
                        break;
                    }
                }
            }

            if (!selected.empty() && selected != last_selected_)
            {
                ROS_INFO("【object_selector】本次选中目标物体: %s", selected.c_str());
                std_msgs::String msg_out;
                msg_out.data = selected;
                result_pub_.publish(msg_out);
                playAudio(selected);
                last_selected_ = selected;
            }
            else
            {
                ROS_WARN("【object_selector】未匹配到目标物体，或已播报过");
            }
        }
    }

    // 拼接向量为字符串
    std::string join(const std::vector<std::string>& v, const std::string& sep)
    {
        std::ostringstream ss;
        for (size_t i = 0; i < v.size(); ++i)
        {
            if (i) ss << sep;
            ss << v[i];
        }
        return ss.str();
    }

    // 写入日志
    void writeDetectLog(const std::vector<std::string>& objs)
    {
        // 时间戳
        std::time_t t = std::time(nullptr);
        char timebuf[32];
        std::strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
        std::ofstream fout(log_path_, std::ios::app);
        if (fout)
        {
            fout << timebuf << " " << join(objs, ",") << std::endl;
        }
        else
        {
            ROS_WARN("【object_selector】无法写入日志文件: %s", log_path_.c_str());
        }
    }

    // 播放音频
    void playAudio(const std::string& obj)
    {
        std::string home = getenv("HOME") ? getenv("HOME") : "";
        std::string audio_path = home + "/ucar_ws/src/speech_command/audio/all/get_" + obj + ".wav";
        std::ifstream fin(audio_path);
        if (fin)
        {
            ROS_INFO("【object_selector】播放语音: get_%s.wav", obj.c_str());
            std::string cmd = "aplay '" + audio_path + "'";
            system(cmd.c_str());
            ros::Duration(0.5).sleep();
            std_msgs::Bool done_msg;
            done_msg.data = true;
            speech_done_pub_.publish(done_msg);
        }
        else
        {
            ROS_WARN("【object_selector】音频文件不存在: %s", audio_path.c_str());
        }
    }
};

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "object_selector");
    ros::NodeHandle nh;
    ObjectSelector selector(nh);
    ros::spin();
    return 0;
}
