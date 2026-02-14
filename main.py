import os

def count_java_lines(folder_path):
    total_lines = 0
    for root, _, files in os.walk(folder_path):
        for file in files:
            if file.endswith(".java"):
                file_path = os.path.join(root, file)
                with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
                    line_count = sum(1 for _ in f)
                    total_lines += line_count
    return total_lines

if __name__ == "__main__":
    # Get the folder where this Python script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    total = count_java_lines(script_dir)
    print(f"Total lines of Java code in '{script_dir}' \n(including subfolders): {total}")
