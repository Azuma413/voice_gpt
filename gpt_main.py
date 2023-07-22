import io, sys
from chatpgt import ChatGPTClass
from voice2text import get_sentens

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')
chatbot = ChatGPTClass()

def main():
    userMessage = get_sentens()
    response = chatbot.chatProcess(userMessage)
    print(response, flush=True)

if __name__=="__main__":
    try:
        while True:
            main()
    except KeyboardInterrupt:
        print("fin")