

/*
for (int t = 0; t < tmpDataPos.size(); t++) {
	//��ʼ������
	if (t == 0) {
		//setEstimate(initObs);
		initFlag = 1;
	}

	//ִ��nnda�Ĳ���
	Update(tmpDataPos.at(t));
	cout << "tmpDataPos.at(t):" << tmpDataPos.at(t) << endl; //tmpData.at(t)Ϊ 2��num�� �ľ���
	cout << "tmpDataPos.at(t).rows():" << tmpDataPos.at(t).rows() << endl; //��
	cout << "tmpDataPos.at(t).cols():" << tmpDataPos.at(t).cols() << endl; //��
	int measureNum = tmpDataPos.at(t).cols();
	for (int i = 0; i < measureNum; i++) {
		for (int j = 0; j < measureNum; j++) {
			MatrixXd d = tmpDataPos.at(t).cols(j) - tmpDataPos.at(t).cols(i);
		}
	}


	initFlag = 0;

	//����Ϊִ��д�����ݵ��ļ�����
	vector<MatrixXd> data;

}
*/